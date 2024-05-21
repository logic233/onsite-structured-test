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
    if True:
        print(*args, **kwargs)
class IDM(PlannerBase):
    def __init__(self, a_bound=7.0, exv=50, t=0.6, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
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
        v, fv, dis_gap, direction = self.getInformFront(state)
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
        aa_max = state[0,6] + 48 * self.dt
        aa_min = state[0,6] - 48 * self.dt
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
        state[:,0] = state[:,0]*direction
        # state[:,2] = state[:,2]*direction
        ego = state[0,:]
        v, fv, dis_gap = ego[2], -1, -1
        
        # 在本车前侧
        x_ind = ego[0] < state[:,0]
        # ? 向前方 沿着x轴平移 会重叠？
        y_ind = (np.abs(ego[1] - state[:,1])) < ((ego[5] + state[:,5])/2)
        # y_ind = ( (ego[1] - state[:,1])**2 + (ego[0] - state[:,0])**2 ) < (15 * v * 0.1) ** 2
        ind = x_ind & y_ind
        if ind.sum() > 0:
            state_ind = state[ind,:]
            front = state_ind[(state_ind[:,0]-ego[0]).argmin(),:]
            fv = front[2]
            dis_gap = front[0] - ego[0] - (ego[4] + front[4])/2
        if dis_gap > 100:
            dis_gap = -1
            fv = -1
        # 看一看前侧 
        # [估计值] 在目力所见 10 内
        theta_d = math.pi / 16
        d_ind = []

        for i in range(len(state)):
            d_ind.append(True)
        for i in range(1, len(state)):
            state_item = state[i]
            # 确保当前速度比所有前车慢
            # 在当前极限刹车距离之外的车均不考虑
            if   math.sqrt((state_item[0] - ego[0])** 2 +   (state_item[1] - ego[1])** 2 ) >  ( ego[2] - state_item[2] ) ** 2 /2 / self.a_bound + ego[2] * 0.5 + 10:
                d_ind[i] = False 
            yaw_2p = get_2p_yaw(ego[0] * direction , ego[1] , state_item[0] * direction, state_item[1])
            if yaw_2p > state[0, 3] + theta_d or yaw_2p < state[0, 3] - theta_d:
                d_ind[i] = False
        ind = x_ind & d_ind
        if ind.sum() > 0:
            state_ind = state[ind,:]
            printf(state_ind)
            printf("+++ 前方的车",)
            front = state_ind[(state_ind[:,2]).argmin(),:]
            printf("###### 最慢的前车")
            printf(front)
            self.exv = max (0.96 * front[2], 0.0001) 
        else:
            self.exv = 50
        printf("###### self.exv fix:", self.exv)
        return v, fv, dis_gap, direction
