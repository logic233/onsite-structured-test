#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import numpy as np
import pandas as pd
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from typing import List, Tuple
from .util.geo import *
from utils.logger import logger
import  math
def printf(*args, **kwargs):
    if False:
        print(*args, **kwargs)
class IDM(PlannerBase):
    def __init__(self, a_bound=4.0, exv=50, t=0.6, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
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
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_ = 0
        
        self.dt = 0

    def init(self, scenario_dict):
        printf("----------------------------IDM INIT----------------------------")
        printf(scenario_dict)
        printf("----------------------------------------------------------------")
        # parse_opendrive(scenario_dict['source_file']['xodr'])

    def act(self, observation: Observation):
        printf("----------------------------IDM ACT----------------------------")
        # printf(observation)
        # printf("----------------------------------------------------------------")      
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
        return [self.deside_acc(state), 0]

    def deside_acc(self, state: pd.DataFrame) :
        v = self.getInformFront(state)
        # if dis_gap < 0:
        #     a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        # else:
        #     # 求解本车与前车的期望距离
        #     # printf(self.s0,self.s1,self.exv,v,self.t)
        #     self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
        #         v - fv) / 2 / (self.a * self.b) ** 0.5
        #     # 求解本车加速度
        #     a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
        if self.exv > v:
                a_idm = self.a_bound
        else:
            a_idm = -self.a_bound
        aa_max = state[0,6] + 30 * self.dt
        aa_min = state[0,6] - 30 * self.dt
        # 对加速度进行约束
        a_idm = np.clip(a_idm, max(-self.a_bound,aa_min), min(self.a_bound , aa_max))
        # printf("#######  v,fv,dis_gap,a_idm ")
        # printf(v,fv,dis_gap,a_idm)
        return a_idm

    def getInformFront(self, state: pd.DataFrame) :
        # direction = np.sign(state[0,2])
        state[0, 3] = range_yaw(state[0, 3])
        if state[0, 3] < np.pi / 2 or state[0, 3] > np.pi * 3 / 2:
            direction = 1.0
        else:
            direction = -1.0
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
        for i in range(1, len(state)):
            state_item = state[i]
            # 重新构建坐标系 s 方向ego车向
            _t , _s = get_xy_in_state(state_item[0] , state_item[1] , ego)
            
            yaw_i = range_yaw(state_item[3])
            yaw_delta = yaw_i - ego[3]
            # 将目标车速度向ego车向分解为
            _vs = math.cos(yaw_delta) * state_item[2]
            _vt = math.sin(yaw_delta) * state_item[2] * (-1)
            distance_2p = math.sqrt(_t** 2 +   _s** 2 ) 

            item_length  = state_item[4]
            item_width = state_item[5]
            #目标车中心到外侧距离
            width_ = math.cos( math.atan(item_length / item_width) - abs(yaw_delta)) * math.sqrt(item_length**2 + item_width ** 2) /2
            # 安全通过 两中心之间水平距离
            safety_cross_width = width_ + ego[5] /2 * 2  
            
            # 需要控制ego 大于正方向车距 
            safety_s = ego[4] /2 + item_length/2 + ( v - _vs ) ** 2 /2 / self.a_bound + v / 2 + 7
            if _s < 0 :
                d_ind[i] = False 
                continue           
            # 在安全车距外的车 忽略
            printf("===========\n",state_item[-1] , safety_s)
            printf("st",round(_s ,2 ) , round(_t ,2 ), "||  vst",round(_vs ,2 ) , round(_vt ,2 ))
            if abs(_s) > safety_s:
                d_ind[i] = False 
                printf("在安全车距外")
                continue                 
            # 在t方向上正在远离ego
            if _t *  _vt > 0  and abs(_t) > safety_cross_width:
                d_ind[i] = False 
                printf("在t方向上正在远离ego")
                continue 
            # t方向上相遇
            t_t = (abs(_t) + safety_cross_width ) / (abs(_vt) + lit_d)
            if v > _vs :
                # s方向上相遇
                d_s =  (_s - item_length /2 - ego[4] /2)
                t_s = get_great_sol(ego[5] /2 , v-_vs , -d_s)
                printf("t_s",t_s,"t_t",t_t)
                # 放宽一点所以 *2
                if t_s != None:
                    if abs(t_s * _vt + _t) > safety_cross_width + t_s  and \
                    abs( (t_s- 0.1) * _vt + _t) > safety_cross_width + t_s  and \
                    abs( (t_s+ 0.1) * _vt + _t) > safety_cross_width + t_s  :
                        d_ind[i] = False 
                        printf("安全超车/汇车")
                        continue    
            
            vsx = _vs - math.sqrt(2 * self.a_bound * ( safety_s - abs(_s)))
            if vsx < exv:
                exv = vsx
                exv_inx = i
        if sum(d_ind) > 0:
            printf("+++ 前方的车",)
            printf(state[d_ind,:][:,-1])
            printf("###### 最慢的前车")
            printf(state[exv_inx][-1])
            self.exv = max (exv, 0.00001) 
        else:
            self.exv = exv_max
        printf("###### self.exv fix:", self.exv)
        return v
