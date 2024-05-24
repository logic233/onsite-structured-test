import math


class updater():
    def __init__(self, length , dt):
        """
        length 车长
        dt 时间间隔
        """
        self.length = length
        
        self.dt = dt

    def u_x(self, x0, v0, yaw0 ):
        return x0 + v0 * self.dt * math.cos(yaw0)

    def u_y(self ,y0, v0, yaw0 ):
        return y0 + v0 * self.dt * math.sin(yaw0)

    def u_yaw(self, yaw0, v0, rot):
        return y0 + v0 * self.dt * math.tan(rot) / self.length

    def u_v(self , v0 , acc):
        return v0 + acc * self.dt


def get_rot_t0(x0 , y0 ,v0 , yaw0 , acc0 , length ,dt):
    up = updater(length , dt)
    
    x1 = up.u_x(x0 ,v0 ,yaw0)
    y1 = up.u_y(y0 ,v0 ,yaw0)
    v1 = up.u_v(v0, acc0)