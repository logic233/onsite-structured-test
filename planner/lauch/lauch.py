#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import numpy as np
import pandas as pd
import math
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive2xml
from utils.opendrive2discretenet import parse_opendrive
from typing import List, Tuple
from utils.ScenarioManager import select_scenario_manager
from utils.ScenarioManager.ScenarioInfo import ScenarioInfo

from utils.opendrive2discretenet.opendriveparser.elements.geometry import Line


class LAUCH(PlannerBase):
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
        # 从visualizer 拿来的
        self.scene_info = None      # 待可视化的场景信息
        self.road_info = None       # 通过parse_opendrive模块解析出的opendrive路网信息
        self.openDriveXml = None

    # 输出(x,y)在哪个road-lanesection-lane中
    def find_in_road_lanesection_lane(self,x,y):
        t_min = float("Inf")
        target = -1
        for road in self.openDriveXml.roads:
            s,t = road._planView.convert_to_reference_coordinates(x,y)
            # 如果有效 那么查一下在当前road 的哪个lane中
            # 可能有 
            # 除非是处在juction中 否则在唯一的lane中
            if (t != None ):
                 # 可能存在多个 roadOffse
                laneOffset_need = None
                t_offset = 0.0
                for laneOffset in road.lanes.laneOffsets:
                    if (s >= laneOffset.start_pos ):
                        laneOffset_need = laneOffset
                if (laneOffset_need != None):
                    t_offset = laneOffset_need.getAttr(s)
                # 可能存在多个 lane_section
                lane_section_need = None
                for lane_section in  road.lanes.lane_sections:
                    if ( s >= lane_section.sPos):
                        lane_section_need = lane_section
                # t为正 左手方向 查left
                lanes_to_search = lane_section_need.leftLanes if(t>=t_offset) else lane_section_need.rightLanes
                
                # 车道外边到中心线距离
                lane_outer_distance = 0.0
                lane_need = None
                # 依次按从内向外顺序查找line
                for lane in lanes_to_search:
                    lane_width_value = 0.0
                    # 处理多端width情况
                    for lane_width in lane.widths:
                        if (s>= lane_section_need.sPos + lane_width.start_pos):
                            # 由于getAttr只会减去自身的sPos
                            lane_width_value = lane_width.getAttr(s-lane_section_need.sPos)
                    lane_outer_distance += lane_width_value
                    # 只要距中心线距离>= 当前外边到中心线距离 就返回
                    if (abs(t-t_offset)<= lane_outer_distance):
                        lane_need = lane
                        break
                if lane_need !=None:
                    print("lane_need",lane_need._parent_road.id,",",lane_section_need.idx,",",lane_need.id)
        


    def init(self, scenario_dict):
        print("---------------------------- INIT----------------------------")
        print(scenario_dict['source_file']['xodr'])
        startPos = scenario_dict['task_info']['startPos']
        targetPos = scenario_dict['task_info']['targetPos']
        
        
        # 返回OpenDrive类的实例对象（经过parser.py解析）
        road_info = parse_opendrive(scenario_dict['source_file']['xodr'])
        self.openDriveXml = parse_opendrive2xml(scenario_dict['source_file']['xodr'])

        print("[startPOS]")
        self.find_in_road_lanesection_lane(startPos[0],startPos[1])
        print("[targetPos] mid")
        self.find_in_road_lanesection_lane((targetPos[0][0]+targetPos[1][0])/2,(targetPos[0][1]+targetPos[1][1])/2)
        
        print("----------------------------------------------------------------")
        # parse_opendrive(scenario_dict['source_file']['xodr'])

    def _load_result_scene(self, mode: str, task: str) -> ScenarioInfo:
        """加载输出文件对应的测试场景"""
        sm = select_scenario_manager(mode, {'tasks': [task]})
        if sm.next():
            return sm.cur_scene
        else:
            raise ValueError(f"Failed to load scenario: {task}")


    def act(self, observation: Observation):
        # 加载主车信息
        frame = pd.DataFrame(
            vars(observation.ego_info),
            columns=['x', 'y', 'v', 'yaw', 'length', 'width'], 
            index=['ego']
        )
        # 加载背景要素状态信息
        for obj_type in observation.object_info:
            for obj_name, obj_info in observation.object_info[obj_type].items():
                sub_frame = pd.DataFrame(vars(obj_info), columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[obj_name])
                frame = pd.concat([frame, sub_frame])
        state = frame.to_numpy()

        return [self.deside_acc(state), 0]

    def deside_acc(self, state: pd.DataFrame) -> float:
        v, fv, dis_gap, direction = self.getInformFront(state)
        # print(v, fv, dis_gap,direction)
        # print(state)
        if dis_gap < 0:
            a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        else:
            # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.t)
            self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
                v - fv) / 2 / (self.a * self.b) ** 0.5
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
        # 对加速度进行约束
        a_idm = np.clip(a_idm, -self.a_bound, 1e7)
        # print(v,fv,dis_gap,a_idm,self.s_)
        # print(state,v,fv,dis_gap,a_idm)
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
