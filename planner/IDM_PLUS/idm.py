#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import numpy as np
import pandas as pd
import  math,json,copy
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from typing import List, Tuple
from utils.logger import logger

from .util.geo import *
from .util.road_info import *
from .util.myopendrive2discretenet import parse_opendrive2xml
from .util.update import *
from .util.lane_graph import *
from .util.pid import *
from .rot import *
# from .junction import *
def printf(*args, **kwargs):
    if False:
        print(*args, **kwargs)
def printf_rot(*args, **kwargs):
    if False:
        printf(*args, **kwargs)
def printf_junction(*args, **kwargs):
    if False:
        printf(*args, **kwargs)
def printf_idm(*args, **kwargs):
    if True:
        printf(*args, **kwargs)

class IDM(PlannerBase):
    def __init__(self, a_bound=5.0, exv=45, t=0.2,a=2.5, b=2.8, gama=4, s0=2.0, s1=1):
        """跟idm模型有关的模型参数
        :param a_bound: 本车加速度绝对值的上下界
        :param exv: 期望速度
        :param t: 反应时间
        :param a: 起步加速度
        :param b: 舒适减速度
        :param gama: 加速度指数
        :param s0: 静止安全距离
        :param s1: 与速度有关的安全距离选择参数
        """
        self.a_bound = a_bound
        self.exv = exv
        self.t = t
        self.a_init = a
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_ = 0
        
        # 更新时间
        self.dt = 0
        # 存储路网xml
        self.openDriveXml = None  
        self.rot = 0
        self.swing_angle = 0
        
        self.tar_x = 0
        self.tar_y = 0
        self.tar_s = 0
        self.tar_t = 0
        self.time_to_tar = 5
        self.junction_id  = None
        self.laneGraph = None
        self.exv_want = None
        
        self.car_name_mem = set()
        self.safety_cross_width_list = [1]
        self.l_st_vst_ego = [1]
    def init(self, scenario_dict):
        printf("----------------------------IDM INIT----------------------------")
        printf(scenario_dict)
        xodr_path = scenario_dict['source_file']['xodr']
        self.openDriveXml = parse_opendrive2xml(xodr_path)
        road_info = parse_opendrive(xodr_path)

        startPos = scenario_dict['task_info']['startPos']
        targetPos = scenario_dict['task_info']['targetPos']
        self.tar_x = (targetPos[0][0]+targetPos[1][0])/2
        self.tar_y = (targetPos[0][1]+targetPos[1][1])/2 
        self.tar_s , self.tar_t = get_st(self.openDriveXml , self.tar_x , self.tar_y)
        start_quad = find_in_road_lanesection_lane( self.openDriveXml, startPos[0],startPos[1])
        target_quad= find_in_road_lanesection_lane( self.openDriveXml , (targetPos[0][0]+targetPos[1][0])/2,(targetPos[0][1]+targetPos[1][1])/2)

        
        self.laneGraph = LaneGraph(road_info,self.openDriveXml)
        self.laneGraph.lane_graph_route(start_quad,target_quad)
        # 将对象转换为格式化的JSON字符串
        # json_str = json.dumps(self.openDriveXml , indent=4)
        printf("----------------------------------------------------------------")
        # parse_opendrive(scenario_dict['source_file']['xodr'])

    def act(self, observation: Observation):
        printf("----------------------------IDM ACT--",observation.test_info["t"],"--------------------------")   
        # 加载主车信息
        frame = pd.DataFrame(
            vars(observation.ego_info),
            columns=['x', 'y', 'v', 'yaw', 'length', 'width','a'], 
            index=['ego']
        )
        # 加载背景要素状态信息
        for obj_type in observation.object_info:
            for obj_name, obj_info in observation.object_info[obj_type].items():
                sub_frame = pd.DataFrame(vars(obj_info), columns=['x', 'y', 'v', 'yaw', 'length', 'width','a'],index=[obj_name])
                sub_frame['name'] = obj_name
                frame = pd.concat([frame, sub_frame])
        state = frame.to_numpy()
        self.dt = observation.test_info["dt"]
        self.rot = observation.ego_info.rot
        
        # x y need to update
        # up = updater(state[0][4] , self.dt)
        # for i in range(len(state)):
        #     state[i][0] = up.u_x(state[i][0] , state[i][2] ,state[i][3])
        #     state[i][1] = up.u_y(state[i][1] , state[i][2] ,state[i][3])
            
            
        quad_this = find_in_road_lanesection_lane(self.openDriveXml, state[0][0] , state[0][1])
        printf("quad_this",quad_this)
        _s , _t = get_st(self.openDriveXml, state[0][0] , state[0][1] , quad_this)
        length =  get_road_length(self.openDriveXml , quad_this)
        self.exv = get_exv(self.openDriveXml , self.laneGraph.get_located_quad())
        # 观察一下前方是否需要减速
        if length != None and _s != None:
            _res_length = length - _s
            for i in range(1,10):
                if self.laneGraph.laneGraph_idx + i < len(self.laneGraph.route_list): 
                    quad = self.laneGraph.route_list[self.laneGraph.laneGraph_idx + i]
                    _l = get_road_length(self.openDriveXml , quad)
                    if _res_length <(state[0][2]**2 - 5**2)/2/1 + 10 :
                        exv_  =  get_exv(self.openDriveXml, quad)
                        self.exv = min(  exv_ , self.exv)
                        _res_length += _l
                    else:
                        break
        # 如果接近终点，直接开冲
        if len(self.laneGraph.route_list) >= 1 and quad_this  ==  self.laneGraph.route_list[-1]:
            yaw_now2tar = get_2p_yaw(state[0][0],state[0][1] ,self.tar_x , self.tar_y)
            if self.tar_s != None:
                k = 1 if self.laneGraph.route_list[-1][2] < 0  else -1
                self.time_to_tar = (self.tar_s - _s)/ (state[0][2] + 1e-6) * k
                time_to_tar_p = self.time_to_tar / (self.time_to_tar + observation.test_info["t"])
                # 比例 * 4
                if time_to_tar_p < 0.05 and abs(yaw_now2tar - state[0][3]) < 1:
                    self.a = self.a_bound 
                else:
                    self.a = self.a_init
        _distance_min = 1e6
    
        # 找到当前所在最近导航quad
        # update [laneGraph_idx]
        # 不会返回None
        _s , _t = get_st(self.openDriveXml, state[0][0] , state[0][1] , self.laneGraph.get_located_quad())
        # 如果_s为None 意味着在当前车道后 或者当前车道前 一直更新idx 直到
        while _s == None:
            res = self.laneGraph.update_route_idx()
            if res == False:
                break
            _s , _t = get_st(self.openDriveXml, state[0][0] , state[0][1] , self.laneGraph.get_located_quad())
        # 考虑变道
        now = self.laneGraph.get_located_quad()
        nex = self.laneGraph.get_located_quad_next_i(1)
        if now != None and nex!=None:
            if (now[0] == nex[0]) and (now[1] == nex[1]):
                res = self.laneGraph.update_route_idx()
                printf("[变道成功]")
        #不太好搞 除非_s 能返回负数
        # length_to_update_next = 5
        # if _s + length_to_update_next >  get_road_length(self.openDriveXml , quad_located):
        #     self.laneGraph.update_route_idx()
        quad_located = self.laneGraph.get_located_quad()
        printf("quad_located" , quad_located)
        # 根据上个周期情况确定
        # if self.exv_want != None:
        #     self.exv = min(self.exv , self.exv_want)
        acc = self.deside_acc(state)
        # self.exv_want = None
        # 带有设定期望时速的副作用
        rot = self.deside_rot(state,acc)
        printf("[self.exv]",self.exv)
        return [acc, rot]
    
    def deside_rot(self, state: pd.DataFrame,acc) :
        quad_located = self.laneGraph.get_located_quad()
        if quad_located == None:
            quad_located = find_in_road_lanesection_lane(self.openDriveXml, state[0][0] , state[0][1])
        _s , _t = get_st(self.openDriveXml, state[0][0] , state[0][1] , quad_located)
        if _s == None:
            return 0
        # 如果在园内 远视
        # if (get_r(self.openDriveXml , _s , self.laneGraph.get_located_quad()) < 20):
        #     printf_rot("远视生效")
        #     _max_time = 3* self.dt
        #     up = updater(state[0][4] , _max_time / 5)
        #     _x_look = state[0][0]
        #     _y_look = state[0][1]
        #     for i in range(5):
        #         _x_look = up.u_x(_x_look , state[0][2] ,state[0][3])
        #         _y_look = up.u_x(_y_look , state[0][2] ,state[0][3])
        #         s_look,t_look = get_st(self.openDriveXml, _x_look , _y_look , quad_located)
        #         if s_look == None:
        #             break
        #         else:
        #             printf_rot("远视",round(_t ,2) ,"->",round(t_look,2))
        #             _s = s_look
        #             _t = t_look
                    
        rot_ans = 0
        # # [test rot]
        yaw_planView = get_heading(self.openDriveXml, state[0][0] , state[0][1],quad_located)
        
        if (yaw_planView == None):
            yaw_planView = state[0][3]
        else:
            yaw_planView =  range_yaw(yaw_planView , 0)
            state[0][3]  = range_yaw( state[0][3] , 0)
            target_t = find_lane_mid_t(self.openDriveXml, _s , quad_located)
            if target_t == None:
                target_t = find_lane_mid_t(self.openDriveXml, _s , quad_located)
            if target_t==None:
                return 0
            printf("target_t",target_t)
            # 在平路上考虑 根据前后方有无车的修正
            change_ = True
            delta = 0
            if state[0][2] < 20:
                consider_distance = 20
            else:
                consider_distance = 30
            if get_r(self.openDriveXml , _s , quad_located) > 40:
                target_t_in = -(target_t - _t)
                for i in range(len(state)):
                    if i == 0:
                        continue
                    printf("check rot" , state[i][-1])
                    i_s , i_t = self.l_st_vst_ego[i][0] , self.l_st_vst_ego[i][1]
                    if i_s == None or i_t == None:
                        continue
                    if  abs(i_s ) < state[0][4] /2 +state[i][4] /2 + 2 :
                        printf("侧向临近车")
                        #  外打
                        if  abs(i_t) < self.safety_cross_width_list[i] + 1 :
                            printf("有临近的车")
                            k = -1 if i_t > _t else 1  
                            t = 1
                            if self.l_st_vst_ego[i][2] * self.l_st_vst_ego[i][1] < 0 :
                                t = 1.5
                            if abs(k * t) > abs(delta):
                                delta = k * t
                    # 正前方忽略
                    if   abs(i_t) < state[0][5] /2 + 0.1 and state[0][2] < 10 and (self.l_st_vst_ego[i][2] * self.l_st_vst_ego[i][1] >= 0):
                        printf("正前方车")
                        continue
                    
                    #  只考虑对应地方
                    if (i_s < consider_distance and i_s > 0) or (i_s <0 and i_s > -consider_distance/2):
                        printf("侧向的车")
                        printf("target_t , i_t , t" , round(target_t_in,2) , round(i_t,2))
                        # target_t = target_t + self.safety_cross_width_list[i] * k
                        #  有车不动
                        if  (target_t_in  < i_t +2 and   i_t-2 < 0)  or    (target_t_in > i_t -2 and  i_t +2 > 0)    :
                            # if a_cc >
                            # tar_want = state[i][2]-1 if state[i][2] > 20 else state[i][2] * 0.9
                            # if state[i][2] > 20:
                            #     self.a_want = 0
                            #     if  self.exv_want != None:
                            #         self.exv_want = min(tar_want , self.exv_want)
                            #     else:
                            #         self.exv_want = tar_want
                            printf("无法向目标行进")
                            change_ = False

                if not change_:
                    target_t = _t + delta

                
                    

            offset_t = get_lane_delta_t(self.openDriveXml, _s , quad_located)
            target_t += offset_t
            printf_rot("_t -> target_t ",_t , target_t)
            offset_t = (target_t - _t)
            # 作为时速调整的副作用
            # if abs(offset_t) > 2:
            #     self.exv_want = self.exv * 0.7
            # elif abs(offset_t) > 1.5:
            #     self.exv_want = self.exv * 0.8
            # elif abs(offset_t) > 1:
            #     self.exv_want = self.exv * 0.9

  
            # [PLAN A] #######################################
            # 考虑t_back 时间能回正
            # t_back = 0.3 + state[0][2] * 0.3
            # _vt_consier = offset_t / t_back 
            
            # [PLAN A END] #######################################
            
            # [PLAN B ] ########################################
            # PID参数
            if state[0][2] > 15:
                Kp = 1
            else:
                Kp = 0.5
            Ki = 0
            Kd = 0

            # 创建PID控制器实例
            pid = PID(Kp, Ki, Kd, setpoint=0)
            feedback = -offset_t
            control = pid.update(feedback, self.dt)
            _vt_consier = control
            # if any( abs(element[0] - state[0][0]) < 15 and abs(element[1] - state[0][1]) < 15    for element in state[1:]):
            #     _vt_consier = 0
            # [PLAN B ] ########################################
            _yaw_consider = math.asin(  np.clip (_vt_consier / (state[0][2] + 1e-7) , -1, 1))
            # printf_rot("_yaw_consider yaw_planView" , _yaw_consider,yaw_planView)
            # 开在左道上 往s减少的方向开
            if quad_located[2] <= 0:
                yaw_future = (_yaw_consider+yaw_planView)
            else:
                yaw_future = (-_yaw_consider+yaw_planView + math.pi)
            yaw_future = range_yaw(yaw_future , 0)
            yaw_now = range_yaw(state[0][3] , 0)
            # printf_rot("yaw_now -> yaw_future", yaw_now , yaw_future)
            _delta_yaw = (yaw_future - yaw_now )
            # 保持差角在 -pi  pi  之间
            if _delta_yaw > math.pi:
                _delta_yaw =  _delta_yaw - 2 * math.pi
            if _delta_yaw < - math.pi:
                _delta_yaw =  _delta_yaw + 2 * math.pi       
            rot_target = math.atan( _delta_yaw / self.dt / (state[0][2]+1e-7) * (state[0][4]/1.7) )
            rot_target = range_yaw(rot_target , -math.pi)
            printf_rot("rot_target 限制前" , rot_target)
            
            # [动力学约束]  -0.7 <= 前轮转角 <= 0.7 
            # [质心偏移角] -0.32 <= 前轮转角 <= 0.32 

            # 5 -> 0.2    10 -> 0.1   12.5 -> 0.05
            kk = 40
            if abs(acc) > 0.5:
                kk = max (20 - (abs(acc) - 0.4) * 20 ,5)  
            if abs(acc) > 1:
                kk = 5
            t0 = 0.25
            if abs(acc) > 0.2:
                t0 = 0.24
            if abs(acc) > 0.5:
                t0 = 0.2
            if abs(acc) > 1:
                t0 = 0.17
            if abs(acc) > 2:
                t0 = 0.1
            if abs(acc) > 3:
                t0 = 0.001
            rot_max =  min (math.atan(kk / ((state[0][2] + 0.5 * self.dt ) **2) ) , t0)

                
            # rot_max = 0.5
            # [动力学约束] -1.4 <= 前轮转速转速 < 1.4
            b1 = 2
            k1 = 10
            t1 = 0.8
            k = min( b1 + k1 / (state[0][2] + 1e-7 ) , t1)
            # k =1
            max_rot_dt =  self.dt * 1.4 * k
            max_rot_a =  self.rot + max_rot_dt 
            min_rot_a =  self.rot - max_rot_dt 
            printf_rot("rot_max :",rot_max)
            printf_rot("rot_a [" ,min_rot_a,max_rot_a ,"]")
            rot_ans = np.clip (rot_target , max(   min_rot_a , -rot_max ), min ( max_rot_a , rot_max ))
              # [摇摆角速度]
            t = 0
            while not check_swing(state[0] , rot_ans , self.dt , self.rot):
                t+=1
                rot_ans = rot_ans * 0.9
            if t!=0:
                printf("check_swing",t)
        printf_rot("rot_ans", rot_ans)
        return rot_ans

    def deside_acc(self, state: pd.DataFrame) :
        v, fv, dis_gap = self.getInformFront(state)
        s,t = get_st(self.openDriveXml , state[0][0] , state[0][1],self.laneGraph.get_located_quad())
        r = get_r(self.openDriveXml , s , self.laneGraph.get_located_quad()) 
        gama = self.gama  if r>15 else self.gama / 2
        gama = self.gama 
        gama = gama if v<20 else gama *2
        idm_t = self.t if v<20 else 0.15
        # a = self.a if r>30 else  self.a/2
        b = self.b if r>30 else self.b / 2
        a_idm1 = 1
        #考虑前方路口减速 让行  
        if s != None :
            # r = 10000
            d = (state[0][2]**2)/b/2 +0.3 * state[0][2] + 5
            length_road = self.laneGraph.get_located_quad()
            # 只有在平路才考虑减速路口
            if r >=30 and length_road != None:
                _length  =  get_road_length(self.openDriveXml ,length_road) - s
                i = 1
                printf_junction("d,_length",d,_length)
                while d >= _length:
                    next_quad = self.laneGraph.get_located_quad_next_i(i)
                    printf_junction("d,_length,next_quad",d,_length,next_quad)
                    if next_quad != None:
                        if state[0][2] < 1 : 
                            t_me = min((_length + 3) / (state[0][2] + 1e-9)  + 0.2, math.sqrt(2 * (_length + 3) / self.a )  + 0.4)
                        else:
                            t_me = (_length + 3) / (state[0][2] + 1e-9) + 0.2
                        printf_junction("开始check_juction",next_quad , t_me)
                        res = self.check_juction(state,next_quad , t_me)
                        _length += get_road_length(self.openDriveXml ,self.laneGraph.get_located_quad_next_i(i))
                        i+=1
                        if not res and res != None:
                            printf_junction("[ATTENTION] 路口危险",next_quad)
                            if v > 2:
                                a_idm1 = -self.b 
                            elif v >1:
                                a_idm1 = -0.9 *self.b
                            elif v > 0.5:
                                a_idm1 = -0.5 *self.b
                            break
                        if res :
                            gama = 10
                            
                    else:
                        break
    
        printf_junction("r",r)
        if dis_gap < 0 :
            a_idm = self.a * (1 - (v / self.exv) ** gama)
        else:
            # 求解本车与前车的期望距离
            # printf(self.s0,self.s1,self.exv,v,idm_t)
            self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + \
                max (idm_t * v + v * (v - fv) / 2 / (self.a * b) ** 0.5 , 0)
            # printf(self.s_  , "=" , self.s0 ,  self.s1 * (v / self.exv) ** 0.5 , idm_t * v , v * (
            #     v - fv) / 2 / (self.a * b) ** 0.5  )
            # if abs (self.tar_y - state[0][1]) < 5 and abs(self.tar_x - state[0][0]) < self.s_ * 4:
            #     dis_gap = 100
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
            printf("self.s_ , dis_gap , a_idm",round(self.s_,2) , round(dis_gap,2),round(a_idm,2))
            
        if a_idm1 < 0 :
            a_idm = a_idm1
        # 较慢加速
        aa_max = state[0,6] + 40 * self.dt
        aa_min = state[0,6] - 40 * self.dt
        # 对加速度进行约束
        a_idm = np.clip(a_idm, max(-self.a_bound,aa_min), min(self.a_bound , aa_max))
        # printf("#######  v,fv,dis_gap,a_idm ")
        printf_idm(v,fv,dis_gap,a_idm)
        return a_idm

    def getInformFront(self, state: pd.DataFrame) :
        state[0, 3] = range_yaw(state[0, 3])
        ego = state[0,:]
        v = ego[2]
        d_ind = []
        for i in range(len(state)):
            d_ind.append(True)    
        # 看一看前侧 
        d_ind[0] = False
        exv = 100000
        exv_max = 50
        exv_inx = -1
        self.l_st_vst_ego = [[0,0,1,0]]
        l_t =[0]
        l_length_ = [1]
        self.safety_cross_width_list = [1] 
        for i in range(1, len(state)):
            state_item = state[i]
            # 重新构建坐标系 s 方向ego车向
            _t , _s = get_xy_in_state(state_item[0] , state_item[1] , ego)
            
            yaw_i = range_yaw(state_item[3])
            yaw_delta = ego[3] - yaw_i  
            # 将目标车速度向ego车向分解为
            _vs = math.cos(yaw_delta) * state_item[2]
            _vt = math.sin(yaw_delta) * state_item[2] 
            s,t = get_st(self.openDriveXml , state[0][0] , state[0][1],self.laneGraph.get_located_quad())
            r = get_r(self.openDriveXml , s , self.laneGraph.get_located_quad()) 
            if(r<30):
                self.l_st_vst_ego.append([_s , _t ,state_item[2] ,state_item[2]])
            else:
                self.l_st_vst_ego.append([_s , _t ,_vs ,_vt])
            distance_2p = math.sqrt(_t** 2 +   _s** 2 ) 

            item_length  = state_item[4]
            item_width = state_item[5]
            #目标车中心到外侧距离
            width_ = math.cos( math.atan(item_length / item_width) - abs(yaw_delta)) * math.sqrt(item_length**2 + item_width ** 2) /2
            # 取代长度 需要检查
            length_ = math.cos(abs(yaw_delta) - math.atan(item_width / item_length) ) * math.sqrt(item_length**2 + item_width ** 2) /2
            l_length_.append(length_)
            # 安全通过 两中心之间水平距离
            safety_cross_width = width_ + ego[5] /2 * 1.2
            self.safety_cross_width_list.append(safety_cross_width)
            # 需要控制ego 大于正方向车距 
            safety_s = ego[4] /2 + item_length/2 + ( v - _vs ) ** 2 /2 / self.a_bound  + ( v - _vs ) * self.dt * 3 + 8 
            # if _s < - (ego[4] /2 + item_length/2 + 1) and abs(_t) < 1:
            #     # 快跑
            #     return ego[2], -1, -1
            if _s < 0 or _vs < 0:
                d_ind[i] = False 
                continue           
            # 在安全车距外的车 忽略
            # printf_idm("===========\n",state_item[-1] , safety_s)
            # printf_idm("st",round(_s ,2 ) , round(_t ,2 ), "||  vst",round(_vs ,2 ) , round(_vt ,2 ))
            # if abs(_s) > safety_s:
            #     d_ind[i] = False 
            #     printf_idm("在安全车距外")
            #     continue                 
            # 在t方向上正在远离ego
        
            # t方向上相遇
            dis_t = _t - safety_cross_width if _t >0 else _t + safety_cross_width
            t_t = (0-dis_t) / (_vt + 1e-7) 
            printf_idm("_t  safety_cross_width dis_t _vt " ,round( _t,2) , round(safety_cross_width,2), round(dis_t,2) ,round(_vt,2) )
            if _t *  _vt >= 0  and (abs(_t) > safety_cross_width):  
                if state[0][2] < 15 or  (abs(_t) > 1 * safety_cross_width):
                    d_ind[i] = False 
                    printf_idm("在t方向上正在远离ego")
                    continue 
            

            d_s =  (_s - item_length /2 - ego[4] /2)
            t_s = get_great_sol(ego[5] /2 , v-_vs , -d_s)
            printf_idm(state_item[-1],"t_t t_s",t_t , t_s)
            want_time = 3
            if state[0][2] >5 :
                want_time = 5
            if state[0][2] >15 :
                want_time = 7         
            if t_t > want_time and  abs(_t) > safety_cross_width:
                d_ind[i] = False 
                printf_idm("保持平行行车")
                continue        
            quad_arr = find_in_road_lanesection_lane_arry(self.openDriveXml ,state_item[0] , state_item[1])
            if no_intersection(quad_arr ,self.laneGraph.route_list )  and (  (abs(_t) > safety_cross_width and abs(_s) > 5) or r < 40 ):
                # printf_idm("not in route",state_item[-1])
                d_ind[i] = False 
                continue
            if v > _vs :
                # s方向上相遇
                # 放宽一点所以 *2
                if t_s != None:
                    if abs(t_s * _vt + _t) > safety_cross_width   and \
                    abs( (t_s+ 0.1) * _vt + _t) > safety_cross_width   and \
                    abs( (t_s+ 0.2) * _vt + _t) > safety_cross_width and (state[0][2] < 15):
                        d_ind[i] = False 
                        printf_idm("安全超车/汇车")
                        continue    
            # vsx = _vs - math.sqrt(2 * self.a_bound * ( safety_s - abs(_s)))
            # if vsx < exv:
            #     exv = vsx
            #     exv_inx = i
        v, fv, dis_gap = ego[2], -1, -1
        if sum(d_ind) > 0:
            # printf_idm("+++ 前方的车",)
            # printf_idm(state[d_ind,:][:,-1])
            idx = d_ind.index(True)
            for i in range(1,len(state)):
                if d_ind[i]:
                    if self.l_st_vst_ego[i][0] - state[i][4]  < self.l_st_vst_ego[idx][0] - state[idx][4]  :
                        idx = i
            fv = self.l_st_vst_ego[idx][2]
            printf_idm("self.l_st_vst_ego[idx][0] - (ego[4] /2 + l_length_[idx]" , round(self.l_st_vst_ego[idx][0],2) , round(ego[4] /2,2)  ,round(l_length_[idx],2))
            dis_gap = self.l_st_vst_ego[idx][0] - (ego[4] /2 + l_length_[idx]) 
            printf_idm("#",state[idx][-1],"# fv dis_gap",fv,dis_gap)
            self.car_name_mem.add(state[idx][-1])
        if dis_gap > 100:
            dis_gap = -1        
            fv = -1
        return v, fv, dis_gap

    def check_juction(self , state, target_quad , t_me):
        if target_quad == None :
            return None
        j_id = None
        for road in self.openDriveXml.roads:
            if road.id == target_quad[0]:
                if road.junction == None or road.length > 25:
                    return None
                else:
                    j_id = road.junction.id
                    break
        if  j_id == None:
            return True
        if self.junction_id != None and self.junction_id  != j_id:
            return True
        self.junction_id = j_id
        conn_arr = []
        for junction in self.openDriveXml.junctions:
            if junction.id == j_id:
                conn_arr = junction._connections

        # 需要领先的距离
        dis_safe = 0
        quad_me_array = []
        for i in range(len(state)):
            x = state[i][0]
            y = state[i][1]
            #  在后方的车
            # yaw_t = get_2p_yaw(x,y,state[0][0],state[0][1])
            # if abs(range_yaw(yaw_t, - math.pi )- range_yaw(state[i][3], - math.pi )) < 0.3:
            #     printf_junction("在后方的车",state[i][-1])
            #     continue
            quad_array = find_in_road_lanesection_lane_arry(self.openDriveXml,x,y)
            if i == 0:
                quad_me_array = quad_array
            if any(item in quad_me_array for item in quad_array):
                continue
            if len(quad_array) != 0:
                for conn in conn_arr:
                    quad_1 = [conn.connectingRoad,0,-1,-1]
                    quad_2 = [conn.connectingRoad,0, 1,-1]
                    for quad in quad_array:
                        s,t = get_st(self.openDriveXml,x,y,quad)
                        yawi = range_yaw(state[i][3]  -math.pi)
                        yaw_view = range_yaw(get_heading(self.openDriveXml,x,y,quad) -math.pi)
                        delta_yaw = abs(yawi - yaw_view)
                        delta_yaw = min (delta_yaw , math.pi *2 - delta_yaw)
                        if delta_yaw > 0.45 or state[i][-1] in self.car_name_mem:
                            printf_junction("已被过滤",state[i][-1],quad)
                            continue
                        # 已有车在路口中
                        if conn.connectingRoad == quad[0] and get_distance(state[0][0],state[0][1] , state[i][0],state[i][1]) < 5:
                            printf_junction("发现已在路口",state[i][-1])
                            # break
                            return False
                        if s == None:
                            continue
                        dis_juction = min(self.laneGraph.get_distance(quad,quad_1) , self.laneGraph.get_distance(quad,quad_2)) - s
                        if dis_juction ==  float('inf'):
                            continue
                        printf_junction(state[i][-1],quad,dis_juction)
                        # 比我快进到路口
                        if (dis_safe+dis_juction)/(state[i][2] + 1e-7)< t_me :
                            printf_junction("发现车快进路口")
                            return False               
        return True