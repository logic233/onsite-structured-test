import math

def check_swing(ego_state,tar_rot , self_dt , self_rot):
    v = ego_state[2] + self_dt* 2  
    r = (v  / (ego_state[4] / 1.7) * math.tan(tar_rot) * self_dt + \
        (math.atan(math.tan(tar_rot) / 2) - math.atan(math.tan(tar_rot) / 2) ) ) * v 
    if abs(r) > 7 * self_dt:
        return False
    return True

class Roll_maker():
    def __init__(self, dt = 0.1):
        self.dt = dt
        self.p = 0
        self.p1 = 0
        self.p2 = 0
        self.w = 0
        self.w1 = 0
        self.w2 = 0
        #  v * v * tan(rot) = wa1
        self.fk = 0.53
    # 获得下一时刻的w的加速度
    def get_wa(rot , v):
        return self.fk  *  v * v * math.tan(rot) 
    def get_rot_target(rot , v):
        wa1 = get_wa(rot , v)
        # 角加速度
        self.w1 = self.w + wa1 * self.dt
        # 角的值
        self.p1 = self.p + self.w * self.dt
        self.p2 = self.p1 + self.w1 * self.dt
        
        _max = ((0.1 - self.p2) /self.dt - self.w1)/ self.dt 
        _min = ((-0.1 - self.p2) /self.dt - self.w1)/ self.dt 
        

        