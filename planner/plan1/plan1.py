#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import math
import numpy as np
import pandas as pd
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from typing import List, Tuple

class plan1(PlannerBase):
    def __init__(self, a_bound=5.0, exv=40, t=1.2, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
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

    def init(self, scenario_dict):
        print("----------------------------plan1 INIT----------------------------")
        global origion, destination,waypoints
        origion = np.array([scenario_dict['task_info']['startPos'][0], scenario_dict['task_info']['startPos'][1]])
        destination = np.array(([scenario_dict['task_info']['targetPos'][0][0], scenario_dict['task_info']['targetPos'][0][1]],
                               [scenario_dict['task_info']['targetPos'][1][0], scenario_dict['task_info']['targetPos'][1][1]]))
        waypoints = scenario_dict['task_info']['waypoints']
        print(origion, destination)
        print(waypoints)
        print("----------------------------------------------------------------")
        # parse_opendrive(scenario_dict['source_file']['xodr'])
        
        global old_x, old_y, nearest_index
        old_x = 0
        old_y = 0
        nearest_index = 2
        # print(nearest_index)

        # 目视距离
        global Ld
        # 车轴距离
        global L
        
    def act(self, observation: Observation):
        # 加载主车信息
        frame = pd.DataFrame(
            vars(observation.ego_info),
            columns=['x', 'y', 'v', 'yaw', 'length', 'width'], 
            index=['ego']
        )

        # 加载背景要素状态信息
        ego=observation.ego_info
        vehicle=observation.object_info['vehicle']
        pedestrian=observation.object_info['pedestrian']
        bicycle=observation.object_info['bicycle']

        global Ld, L,waypoints,nearest_index
        # 配置全局信息
        L=ego.length*0.8
        # 目视距离使用ld=k*v+a的方式
        Ld=0.4*ego.v+0.1



        # test
        print("----------------------------plan1 ACT----------------------------")
        # 加载背景要素状态信息
        print("+++++++++observation+++++++++")
        print(observation)
        # print("+++++++++waypoints+++++++++")
        # print(waypoints)
        print("+++++++++L and Ld and npi+++++++++")
        # if(nearest_index<2):
        #     nearest_index = 3
        print(L, Ld, nearest_index)
        # self.check(observation)


        
        for obj_type in observation.object_info:
            for obj_name, obj_info in observation.object_info[obj_type].items():
                sub_frame = pd.DataFrame(vars(obj_info), columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[obj_name])
                frame = pd.concat([frame, sub_frame])
        state = frame.to_numpy()

        # 查看输出值
        # print("ego.v",ego.v)
        a=self.deside_acc(ego)
        theta=self.getSteeringAngle(ego)
        print("+++++++++返回值+++++++++")
        print(a, theta)

        return [a, theta]
    

    # 获取前轮转角
    def getSteeringAngle(self, ego) -> float:
        # global nearest_index
        # print(nearest_index)
        gx, gy = self.getAttributePoint(ego.x, ego.y)
        return self.pure_pursuit_control(gx, gy, ego.x, ego.y, ego.yaw)


    def deside_acc(self, ego) -> float:
        # print("ego.v",ego.v)
        a_idm=1.55
        if(ego.v>15):
            a_idm=-0.5
        # v, fv, dis_gap, direction = self.getInformFront(state)
        # if dis_gap < 0:
        #     a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        # else:
        #     # 求解本车与前车的期望距离
        #     # print(self.s0,self.s1,self.exv,v,self.t)
        #     self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
        #         v - fv) / 2 / (self.a * self.b) ** 0.5
        #     # 求解本车加速度
        #     a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
        # # 对加速度进行约束
        # a_idm = np.clip(a_idm, -self.a_bound, 1e7)
        return a_idm

    def getInformFront(self, state: pd.DataFrame) -> Tuple[float, float, float, float]:
        # direction = np.sign(state[0,2])
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
        y_ind = (np.abs(ego[1] - state[:,1])) < ((ego[5] + state[:,5])/2)
        ind = x_ind & y_ind
        if ind.sum() > 0:
            state_ind = state[ind,:]
            front = state_ind[(state_ind[:,0]-ego[0]).argmin(),:]
            # print(front)
            fv = front[2]
            dis_gap = front[0] - ego[0] - (ego[4] + front[4])/2
        if dis_gap > 100:
            dis_gap = -1
            fv = -1
        return v, fv, dis_gap, direction

    def check(self, observation: Observation):
        global old_x, old_y
        if(old_x!=0 and old_y!=0 and observation.ego_info.x != old_x):
            k = (observation.ego_info.y - old_y) / (observation.ego_info.x - old_x)
            theta = np.arctan(k)
            print("----------theta----------")
            print("theta", theta)
            print(math.pi+theta)
            print("yaw", observation.ego_info.yaw)
        old_x = observation.ego_info.x
        old_y = observation.ego_info.y


    # 获得目标点
    def getAttributePoint(self, x, y) -> Tuple[float, float]:
        # 目视距离
        # 获取最接近目视距离的目标点
        global nearest_index,Ld,waypoints
        # print("-----------nearest_index and Ld-----------")
        # print(nearest_index, Ld)
        while(self.getDistance(x, y, nearest_index) < Ld and (nearest_index < len(waypoints))):
            nearest_index += 1
        if(nearest_index >= len(waypoints)):
            return [waypoints[np.str(nearest_index-1)][0], waypoints[np.str(nearest_index-1)][1]]
        # if(nearest_index > 1 and self.getDistance(x, y, nearest_index)-Ld > Ld-self.getDistance(x, y, nearest_index-1)):
        #     nearest_index -= 1
        return [waypoints[np.str(nearest_index)][0], waypoints[np.str(nearest_index)][1]] 

    # 计算车身位置与目标点的距离
    def getDistance(self, x, y, index):
        # 数字索引转化为字符串索引
        # 目标点
        global waypoints
        # print("-----------index-----------")
        # print(index)
        tx = waypoints.get(str(index))[0]
        ty = waypoints.get(str(index))[1]
        dx = tx - x
        dy = ty - y
        return np.sqrt(dx**2 + dy**2)

    # 轨迹跟踪算法————计算前轮转角的pure_pursuit控制
    def pure_pursuit_control(self, gx, gy, x, y, yaw) -> float:
        print("-----------------pure_pursuit_control-----------------")
        print("方向、yaw",np.arctan2(gy-y,gx-x) , yaw)
        alpha = np.arctan2(gy-y,gx-x) - yaw
        # 车轴距离
        global L
        # 目视距离
        global Ld
        # 车轴半径R
        R=Ld/(2*np.sin(alpha))
        print("L,R",L,R)
        # 前轮转角
        delta = np.arctan(L/R)
        print("delta",delta)
        return delta
    
    


