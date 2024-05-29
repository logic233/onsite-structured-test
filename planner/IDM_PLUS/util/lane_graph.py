from .myopendrive2discretenet import parse_opendrive,parse_opendrive2xml
import numpy as np
import copy
import scipy
import random
from .road_info import *

class LaneGraph():
    def __init__(self, road_info ,openDriveXml):
        self.lane_quad_list = []
        self.route_list = []
        self.laneGraph_idx = 0
        self.lane_graph = None
        self.openDriveXml = openDriveXml  
        self.lane_graph_init(road_info)

    def lane_quad_maker(self,s):
        ret =[]
        for i in s.split('.'):
            if i=='None':
                return None
            else:
                ret.append(int(i))
        return ret
    def lane_graph_init(self,road_info):
        for dis_lane in road_info.discretelanes:
            lane_quad = self.lane_quad_maker(dis_lane.lane_id) 
            if lane_quad==None:
                continue
            self.lane_quad_list.append(lane_quad)
        
        d_arr = np.zeros((len(self.lane_quad_list), len(self.lane_quad_list)))
        # 根据dis_lane.successor 构造边 权值给10
        # successor_coneect_weight = 10
        for dis_lane in road_info.discretelanes:
            for succ_id in dis_lane.successor:
                from_quad = self.lane_quad_maker(dis_lane.lane_id)
                to_quad = self.lane_quad_maker(succ_id)
                if(to_quad == None):
                    continue
                try:
                    from_idx = self.lane_quad_list.index(from_quad)
                    to_idx = self.lane_quad_list.index(to_quad)
                except ValueError:
                    continue
                d_arr[from_idx][to_idx] = get_road_length(self.openDriveXml,from_quad)
                
        # 相近车道 也构建边 权值 为100
        neighbor_lane_weight = 100 
        for lane_quad in self.lane_quad_list:
            lane_quad_for_lane = lane_quad[2]
            # 中心线应该不会出现 因此直接忽略
            if lane_quad_for_lane == 0 :
                continue
            flag = 1 if lane_quad_for_lane>0 else -1
            
            lane_quad_target  = copy.copy(lane_quad)
            # 通过外边找内边 并构建双向
            if (abs(lane_quad_for_lane) > 1):
                lane_quad_target[2]-=flag
                if not (lane_quad_target in self.lane_quad_list):
                    continue
                from_idx = self.lane_quad_list.index(lane_quad)
                to_idx = self.lane_quad_list.index(lane_quad_target)
                d_arr[from_idx][to_idx] = neighbor_lane_weight
                d_arr[to_idx][from_idx] = neighbor_lane_weight
        d_arr_csr_matrix=scipy.sparse.csr_matrix(d_arr)
        self.lane_graph = d_arr_csr_matrix
    
    def lane_graph_route(self,start_quad,target_quad):
        if start_quad==None or target_quad == None:
            return 
        start_idx = self.lane_quad_list.index(start_quad)
        target_idx = self.lane_quad_list.index(target_quad)
        self.lane_graph_route_idx(start_idx,target_idx)

    def lane_graph_route_idx(self,start_idx,target_idx):
        target_quad = self.lane_quad_list[target_idx]
        dist_matrix, predecessors = scipy.sparse.csgraph.dijkstra(self.lane_graph, return_predecessors=True, indices=start_idx)

        # 存储从起点开始的路径集合
        route_list = [target_quad]
        pre_idx = target_idx
        while True:
            pre_idx = predecessors[pre_idx]
            if pre_idx < 0 or pre_idx>len(self.lane_quad_list):
                # print("route_list [NO]")
                break
            route_list.insert(0,self.lane_quad_list[pre_idx])
            if (pre_idx == start_idx):
                break
        # print("###############[route_list]###############")
        # print(route_list)
        self.route_list = route_list
    
    def get_distance(self,start_quad,target_quad):
        if not start_quad in self.lane_quad_list or not target_quad in self.lane_quad_list:
            return float("inf")
        start_idx = self.lane_quad_list.index(start_quad)
        target_idx = self.lane_quad_list.index(target_quad)
        dist_matrix, predecessors = scipy.sparse.csgraph.dijkstra(self.lane_graph, return_predecessors=True, indices=start_idx)
        return dist_matrix[target_idx]
    def get_located_quad(self):
        if self.laneGraph_idx >=0 and self.laneGraph_idx<len(self.route_list):
            return self.route_list[self.laneGraph_idx]
        else :
            return None
    def get_located_quad_next_i(self,i):
        if self.laneGraph_idx + i >=0 and self.laneGraph_idx + i<len(self.route_list):
            return self.route_list[self.laneGraph_idx+ i] 
        else :
            return None
    def update_route_idx(self):
        if self.laneGraph_idx+1 >=0 and self.laneGraph_idx + 1<len(self.route_list):
            self.laneGraph_idx +=1 
            return True
        return False
    
if __name__ == '__main__':
    xodr_path = r'D:\project_s\onsite\onsite-structured-test\scenario\replay\0_79_merge_81\0_79_merge_81.xodr'
    road_info = parse_opendrive(xodr_path)
    openDriveXml = parse_opendrive2xml(xodr_path)
    laneGraph = LaneGraph(road_info,openDriveXml)
    length = len(laneGraph.lane_quad_list)
    mem = []
    for i in range(100):
        print(i)
        while True:
            start_idx = random.randint(0, length-1)
            start_idx = 0
            end_idx = random.randint(0, length-1)
            # end_idx = 39
            p = (start_idx,end_idx)
            print(p)
            if not (p in mem):
                mem.append(p)
                print(laneGraph.lane_quad_list[start_idx] , "-->" , laneGraph.lane_quad_list[end_idx])
                laneGraph.lane_graph_route_idx(start_idx,end_idx)
                print(laneGraph.get_distance(laneGraph.lane_quad_list[start_idx] , laneGraph.lane_quad_list[end_idx]))
                break 
        
        
        s = input()
        
    
    # lane 拥有 lane_id 属性，为字符串形如1.0.-2.-1 
    # 含义为Road 1 的 laneSection 0 的 lane -2  最后一位均为-1 含义未知
    # 这儿将其转为 [1,0,-2,-1]形式 并依次存入self.dis_lane_id_list中
  