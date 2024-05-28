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
    if True:
        print(*args, **kwargs)
def printf_rot(*args, **kwargs):
    if False:
        printf(*args, **kwargs)
class IDM(PlannerBase):
    def __init__(self, a_bound=5.0, exv=45, t=0.2, a=2.5, b=2.8, gama=1, s0=4.0, s1=0.5):
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
        
        self.laneGraph = None
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
        self.exv = get_exv(self.openDriveXml , quad_this)
        # 观察一下前方是否需要减速
        if length != None and _s != None:
            _exv_sum = 0
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
                if time_to_tar_p < 0.1 and abs(yaw_now2tar - state[0][3]) < 1:
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
        #不太好搞 除非_s 能返回负数
        # length_to_update_next = 5
        # if _s + length_to_update_next >  get_road_length(self.openDriveXml , quad_located):
        #     self.laneGraph.update_route_idx()
        quad_located = self.laneGraph.get_located_quad()
        printf("quad_located" , quad_located)
        # 带有设定期望时速的副作用
        rot = self.deside_rot(state)
        printf("[self.exv]",self.exv)
        return [self.deside_acc(state), rot]
    
    def deside_rot(self, state: pd.DataFrame) :
        quad_located = self.laneGraph.get_located_quad()
        if quad_located == None:
            quad_located = find_in_road_lanesection_lane(self.openDriveXml, state[0][0] , state[0][1])
        _s , _t = get_st(self.openDriveXml, state[0][0] , state[0][1] , quad_located)
        if _s == None:
            return 0
        # _max_time = 1
        # up = updater(state[0][4] , _max_time / 5)
        # _x_look = state[0][0]
        # _y_look = state[0][1]
        # for i in range(5):
        #     _x_look = up.u_x(_x_look , state[0][2] ,state[0][3])
        #     _y_look = up.u_x(_y_look , state[0][2] ,state[0][3])
        #     s_look,t_look = get_st(self.openDriveXml, _x_look , _y_look , quad_located)
        #     if s_look == None:
        #         break
        #     else:
        #         _s = s_look
        #         _t = t_look
        
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
            offset_t = get_lane_delta_t(self.openDriveXml, _s , quad_located)
            target_t += offset_t
            printf_rot("_t -> target_t ",_t , target_t)
            offset_t = (target_t - _t)
            # 作为时速调整的副作用
            if abs(offset_t) > 2:
                self.exv = self.exv * 0.5
            elif abs(offset_t) > 1.5:
                self.exv = self.exv * 0.65
            elif abs(offset_t) > 1:
                self.exv = self.exv * 0.8

  
            # [PLAN A] #######################################
            # 考虑t_back 时间能回正
            # t_back = 0.3 + state[0][2] * 0.3
            # _vt_consier = offset_t / t_back 
            
            # [PLAN A END] #######################################
            
            # [PLAN B ] ########################################
            # PID参数
            Kp = 0.3
            Ki = 0
            Kd = 0

            # 创建PID控制器实例
            pid = PID(Kp, Ki, Kd, setpoint=0)
            feedback = -offset_t
            control = pid.update(feedback, self.dt)
            _vt_consier = control 
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
            kk = 5
            t0 = 0.2
            rot_max =  min (math.atan(kk / ((state[0][2]  + self.a * self.dt) **2) ) , t0)
            # rot_max = 0.5
            # [动力学约束] -1.4 <= 前轮转速转速 < 1.4
            b1 = 0.1 
            k1 = 10
            t1 = 0.8
            k = min( b1 + k1 / (state[0][2]  + self.a * self.dt) ** 2, t1)
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
                print("check_swing",t)
        printf_rot("rot_ans", rot_ans)
        return rot_ans

    def deside_acc(self, state: pd.DataFrame) :
        v, fv, dis_gap = self.getInformFront(state)
        if dis_gap < 0 :
            a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        else:
            # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.t)
            self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + \
                max (self.t * v + v * (v - fv) / 2 / (self.a * self.b) ** 0.5 , 0)
            # printf(self.s_  , "=" , self.s0 ,  self.s1 * (v / self.exv) ** 0.5 , self.t * v , v * (
            #     v - fv) / 2 / (self.a * self.b) ** 0.5  )
            # if abs (self.tar_y - state[0][1]) < 5 and abs(self.tar_x - state[0][0]) < self.s_ * 4:
            #     dis_gap = 100
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
            
        #考虑前方路口减速 让行
        s,t = get_st(self.openDriveXml , state[0][0] , state[0][1])
        if s!=None:
            if get_road_length(self.openDriveXml ,self.laneGraph.get_located_quad()) - s <= (state[0][2]**2)/self.b/2 + 3:
                next_quad = self.laneGraph.get_located_quad_next_i(1)
                if next_quad != None:
                    printf("开始check_juction",next_quad)
                    res = self.check_juction(state,next_quad)
                    if not res:
                        printf("[ATTENTION] 即将有车驶入路口",next_quad)
                        a_idm = -self.b
        # 较慢加速
        aa_max = state[0,6] + 10 * self.dt
        aa_min = state[0,6] - 30 * self.dt
        # 对加速度进行约束
        a_idm = np.clip(a_idm, max(-self.a_bound,aa_min), min(self.a_bound , aa_max))
        # printf("#######  v,fv,dis_gap,a_idm ")
        printf(v,fv,dis_gap,a_idm)
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
        l_st_vst = [[0,0,1,0]]
        l_length_ = [1]
        l_safety_s = [1]
        for i in range(1, len(state)):
            state_item = state[i]
            # 重新构建坐标系 s 方向ego车向
            _t , _s = get_xy_in_state(state_item[0] , state_item[1] , ego)
            
            yaw_i = range_yaw(state_item[3])
            yaw_delta = yaw_i - ego[3]
            # 将目标车速度向ego车向分解为
            _vs = math.cos(yaw_delta) * state_item[2]
            _vt = math.sin(yaw_delta) * state_item[2] 
            l_st_vst.append([_s , _t ,_vs ,_vt])
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
            
            # 需要控制ego 大于正方向车距 
            safety_s = ego[4] /2 + item_length/2 + ( v - _vs ) ** 2 /2 / self.a_bound  + ( v - _vs ) * self.dt * 3 + 8 
            l_safety_s.append(safety_s)
            if _s < - (ego[4] /2 + item_length/2 + 1) and abs(_t) < 1:
                # 快跑
                return ego[2], -1, -1
            if _s < 0 :
                d_ind[i] = False 
                continue           
            # 在安全车距外的车 忽略
            # printf("===========\n",state_item[-1] , safety_s)
            # printf("st",round(_s ,2 ) , round(_t ,2 ), "||  vst",round(_vs ,2 ) , round(_vt ,2 ))
            # if abs(_s) > safety_s:
            #     d_ind[i] = False 
            #     printf("在安全车距外")
            #     continue                 
            # 在t方向上正在远离ego
            if _t *  _vt >= 0  and abs(_t) > safety_cross_width:
                d_ind[i] = False 
                # printf("在t方向上正在远离ego")
                continue 
            # t方向上相遇
            t_t = (abs(_t) + safety_cross_width ) / (abs(_vt) + 1e-7)
            # printf(state_item[-1],"t_t",t_t)
            if t_t > 20 and  abs(_t) > safety_cross_width:
                d_ind[i] = False 
                # printf("保持平行行车")
                continue           
            if v > _vs :
                # s方向上相遇
                d_s =  (_s - item_length /2 - ego[4] /2)
                t_s = get_great_sol(ego[5] /2 , v-_vs , -d_s)
                # printf("t_s",t_s)
                # 放宽一点所以 *2
                if t_s != None:
                    if abs(t_s * _vt + _t) > safety_cross_width + t_s  and \
                    abs( (t_s- 0.1) * _vt + _t) > safety_cross_width + t_s  and \
                    abs( (t_s+ 0.1) * _vt + _t) > safety_cross_width + t_s  :
                        d_ind[i] = False 
                        # printf("安全超车/汇车")
                        continue    
            # vsx = _vs - math.sqrt(2 * self.a_bound * ( safety_s - abs(_s)))
            # if vsx < exv:
            #     exv = vsx
            #     exv_inx = i
        v, fv, dis_gap = ego[2], -1, -1
        if sum(d_ind) > 0:
            # printf("+++ 前方的车",)
            # printf(state[d_ind,:][:,-1])
            idx = d_ind.index(True)
            for i in range(1,len(state)):
                if d_ind[i]:
                    if l_st_vst[i][0] < l_st_vst[idx][0]:
                        idx = i
            fv = l_st_vst[idx][2]
            dis_gap = l_st_vst[idx][0] - (ego[4] /2 + l_length_[idx]) 
            # printf("#",state[idx][-1],"#",fv,dis_gap)
        if dis_gap > 100:
            dis_gap = -1        
            fv = -1
        return v, fv, dis_gap

    def check_juction(self , state, target_quad):
        if target_quad == None :
            return True
        j_id = None
        for road in self.openDriveXml.roads:
            if road.id == target_quad[0]:
                if road.junction == None:
                    return True
                else:
                    j_id = road.junction.id
                    break
        if  j_id == None:
            return True
        conn_arr = []
        for junction in self.openDriveXml.junctions:
            if junction.id == j_id:
                conn_arr = junction._connections

        # 需要领先的距离
        dis_safe = 0
        quad_me_array = []
        t_me = 1
        for i in range(len(state)):
            x = state[i][0]
            y = state[i][1]
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
                        if delta_yaw > 0.4:
                            printf("已被过滤",state[i][-1],quad)
                        # 已有车在路口中
                        # if conn.connectingRoad == quad[0]:
                        #     printf("发现已在路口",state[i][-1])
                        #     return False
                        if s == None:
                            continue
                        dis_juction = min(self.laneGraph.get_distance(quad,quad_1) , self.laneGraph.get_distance(quad,quad_2)) - s
                        if dis_juction ==  float('inf'):
                            continue
                        printf(state[i][-1],quad,dis_juction)
                        # 比我快进到路口
                        if (dis_safe+dis_juction)/(state[i][2] + 1e-7)< t_me:
                            printf("发现车快进路口")
                            return False               
        return True