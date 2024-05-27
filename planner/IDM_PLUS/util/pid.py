class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, feedback_value, dt):
        # 计算误差
        error = self.setpoint - feedback_value
        # 计算积分项
        self.integral += error * dt
        # 计算微分项
        derivative = (error - self.previous_error) / dt
        # 计算输出
        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * derivative
        # print("PID" , round(P,2) , round(I,2) , round(D,2))
        output = P  + I + D 
        # 更新前一误差
        self.previous_error = error
        return output
if __name__ == "__main__":
    import time

    # PID参数
    Kp = 1.0
    Ki = 0.1
    Kd = 0.05

    # 创建PID控制器实例
    pid = PID(Kp, Ki, Kd, setpoint=10)

    # 模拟反馈值
    feedback = 0
    dt = 0.1  # 时间间隔

    for _ in range(100):
        # 获取控制器输出
        control = pid.update(feedback, dt)
        # 模拟系统响应，假设系统是一个简单的积分器
        feedback += control * dt
        # 打印当前反馈值
        print(f"Feedback: {feedback:.2f}")
        # 等待时间间隔
        time.sleep(dt)