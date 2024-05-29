import math
def get_distance(x,y,x1,y1):
    return math.sqrt((x-x1)**2 + (y-y1)**2)
def range_yaw(yaw , _min = 0):
    if yaw > (_min + 2 * math.pi) :
        return range_yaw(yaw - 2 * math.pi , _min)
    if yaw < _min :
        return range_yaw(yaw + 2 * math.pi , _min)
    return yaw
def get_2p_yaw(x1 , y1 ,x2 ,y2):
    if x2 - x1 == 0:
        # 垂直线的特殊情况
        return math.pi /2  if y2 > y1 else math.pi /2 * 3
    costheta = (x2 - x1) / math.sqrt( (x1 -x2)**2  + (y1 - y2)**2 )
    
    
    # 计算夹角（弧度）
    theta = math.acos(costheta)   
    if y1 > y2 :
        theta =  math.pi * 2 - theta
    return range_yaw(theta)

def get_xy_in_state(x,y , state):
    """将点 (x, y) 逆时针旋转指定角度"""
    #  columns=['x', 'y', 'v', 'yaw', 'length', 'width','a']
    # print(get_xy_in_state, x , y , state)
    angle_radians = state[3] - math.pi / 2
    cos_theta = math.cos(angle_radians)
    sin_theta = math.sin(angle_radians)
    x -=  state[0]
    y -= state[1]
    # print(x , y)

    x_new = x * cos_theta + y * sin_theta 
    y_new =  y * cos_theta - x * sin_theta 
    return x_new, y_new   
def get_great_sol(a,b,c):
    d = (b**2) - (4*a*c)
    if d < 0:
        return None
    else:
        return (-b+math.sqrt(d))/(2*a)


def no_intersection(list1, list2):
    if list1 == None or list2 == None:
        return True
    for sublist1 in list1:
        for sublist2 in list2:
            if sublist1 == sublist2:
                return False
    return True

lit_d = 0.0001

if __name__ == '__main__':
    print(get_2p_yaw(138.472 , 113.175 , 134.88 , 105.41))