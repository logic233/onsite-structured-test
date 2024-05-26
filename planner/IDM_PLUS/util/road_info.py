import math
# 找到指定四元组
def find_in_road_lanesection_lane(openDriveXml,x,y):
    t_min = float("Inf")
    target = -1
    for road in openDriveXml.roads:
        s,t = road._planView.convert_to_reference_coordinates(x,y)
        # 如果有效 那么查一下在当前road 的哪个lane中
        # 可能有 
        # 除非是处在juction中 否则在唯一的lane中
        if (t != None and s > 0):
                # 可能存在多个 roadOffse
            laneOffset_may = None
            t_offset = 0.0
            for laneOffset in road.lanes.laneOffsets:
                if (s >= laneOffset.start_pos ):
                    laneOffset_may = laneOffset
            if (laneOffset_may != None):
                t_offset = laneOffset_may.getAttr(s)
            # 可能存在多个 lane_section
            lane_section_may = None
            for lane_section in  road.lanes.lane_sections:
                if ( s >= lane_section.sPos):
                    lane_section_may = lane_section
            # t为正 左手方向 查left
            lanes_to_search = lane_section_may.leftLanes if(t>=t_offset) else lane_section_may.rightLanes
            
            # 车道外边到中心线距离
            lane_outer_distance = 0.0
            lane_may = None
            # 依次按从内向外顺序查找line
            for lane in lanes_to_search:
                lane_width_value = 0.0
                # 处理多端width情况
                for lane_width in lane.widths:
                    if (s>= lane_section_may.sPos + lane_width.start_pos):
                        # 由于getAttr只会减去自身的sPos
                        lane_width_value = lane_width.getAttr(s-lane_section_may.sPos)
                lane_outer_distance += lane_width_value
                # 只要距中心线距离>= 当前外边到中心线距离 就返回
                if (abs(t-t_offset)<= lane_outer_distance):
                    lane_may = lane
                    break
            if lane_may !=None:
                ret = [lane_may._parent_road.id,lane_section_may.idx,lane_may.id,-1]
                # print("lane_may",ret)
                return ret
def get_st(openDriveXml,x,y,quad=None):
    if quad == None:
        quad = find_in_road_lanesection_lane(openDriveXml,x,y)
    if quad == None :
        return None,None
    for road in openDriveXml.roads:
        if road.id == quad[0]:
            return road._planView.convert_to_reference_coordinates(x,y)
        
def get_road_length(openDriveXml,quad):
    if quad == None:
        return None
    for road in openDriveXml.roads:
        if road.id == quad[0]:
            return road.length
    return None      
def get_distance_to_quad(openDriveXml ,x,y,quad):
    _s , _t = get_st(openDriveXml, x , y , quad)
    if quad == None:
        return None
    if _s == None:
        return None
    ret = 0
    for road in openDriveXml.roads:
        if road.id == quad[0]:
            if _s <0:
                ret += _s**2
            if _s > road.length:
                ret += (_s-road.length)**2
            ret += _t **2
    return math.sqrt(ret)                  
               
def get_heading(openDriveXml,x,y,quad=None):
    if quad == None:
        quad = find_in_road_lanesection_lane(openDriveXml,x,y)
    if quad == None :
        return None
    for road in openDriveXml.roads:
        if road.id == quad[0]:
            s,t = get_st(openDriveXml,x,y,quad)
            t = 0
            if quad[2] > 0:
                t -= math.pi 
            return road._planView.get_hed(s) + t
        

def find_lane_mid_t(openDriveXml,s,lane_quad):
    if lane_quad==None:
        return None
    if s == None:
        return None
    for road in openDriveXml.roads:
        if road.id == lane_quad[0]:
            laneOffset_may = None
            t_offset = 0.0
            for laneOffset in road.lanes.laneOffsets:
                if (s >= laneOffset.start_pos ):
                    laneOffset_may = laneOffset
            if (laneOffset_may != None):
                t_offset = laneOffset_may.getAttr(s)
            
            lanes_section = road._lanes._lane_sections[lane_quad[1]]
            lanes_array = lanes_section.leftLanes if(lane_quad[2] > 0 ) else lanes_section.rightLanes
            lane_outer_distance = 0.0
            for lane in  lanes_array:
                lane_width_value = 0.0
                # 处理多端width情况
                for lane_width in lane.widths:
                    if (s>= lanes_section.sPos + lane_width.start_pos):
                        # 由于getAttr只会减去自身的sPos
                        lane_width_value = lane_width.getAttr(s-lanes_section.sPos)
                lane_outer_distance += lane_width_value
                # 只要距中心线距离>= 当前外边到中心线距离 就返回
                if (lane.id == lane_quad[2]):
                    if lane.id > 0 :
                        return t_offset+(lane_outer_distance-lane_width_value/2)
                    else:
                        return t_offset-(lane_outer_distance-lane_width_value/2)
                    break   
    return None

def get_lane_delta_t(openDriveXml,s,lane_quad):
    if lane_quad == None:
        return 0
    for road in openDriveXml.roads:
        if road.id == lane_quad[0]:
            cur = road._planView.get_curvature(s)
            if cur ==0:
                return 0
            r = abs(1/cur)
            k = 1 if cur>0 else -1
            if r < 30:
                return k * 0.6
            return k * 0.2
    return 0
def get_exv(openDriveXml,s,lane_quad):
    max_exv = 45
    if lane_quad == None:
        return max_exv
    for road in openDriveXml.roads:
        if road.id == lane_quad[0]:
            cur = road._planView.get_curvature(s)
            if cur ==0:
                return max_exv
            r = abs(1/cur)
            if r < 30:
                return 9
            if r < 50:
                return 15
    return max_exv 