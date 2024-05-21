import math

def range_yaw(yaw):
    if yaw > 2 * math.pi :
        return range_yaw(yaw - 2 * math.pi)
    if yaw < 0 :
        return range_yaw(yaw + 2 * math.pi)
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


if __name__ == '__main__':
    print(get_2p_yaw(138.472 , 113.175 , 134.88 , 105.41))