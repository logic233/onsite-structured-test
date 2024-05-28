# from .myopendrive2discretenet import parse_opendrive
from .util.myopendrive2discretenet import parse_opendrive
from .util.myopendrive2discretenet import parse_opendrive2xml
from .util.road_info import *



def check_juction(openDriveXml, state, target_quad):
    if target_quad == None :
        return True
    j_id = None
    for road in openDriveXml.roads:
        if road.id == target_quad[0]:
            if road.junction == None:
                return True
            else:
                j_id = road.junction.id
                break
    if  j_id == None:
        return True
    conn_arr = []
    for junction in openDriveXml.junctions:
        if junction.id == j_id:
            conn_arr = junction._connections
    dis_juction_me = 10
    dis_safe = 10
    for i in range(len(state)):
        x = state[i][0]
        y = state[i][1]
        quad = find_in_road_lanesection_lane(openDriveXml,x,y)
        if quad != None:
            for conn in conn_arr:
                # 已有车在路口中
                if conn.connectingRoad == quad[0]:
                    return False
                if conn.incomingRoad == quad[0]:
                    s,t = get_st(openDriveXml,x,y,quad)
                    if s == None:
                        return True
                    dis_juction = get_road_length(openDriveXml,quad) - s
                    if i==0:
                        dis_juction_me = dis_juction
                    #  多看
                    else:
                        # 比我快进到路口
                        if dis_safe+dis_juction<dis_juction_me:
                            return False
    return True


if __name__ == '__main__':
    xodr_path = r'D:\project_s\onsite\onsite-structured-test\scenario\replay\0_79_merge_81\0_79_merge_81.xodr'
    openDriveXml = parse_opendrive2xml(xodr_path)
    print(openDriveXml)
        
    
