# controllers.py

class PIDController:
    """既存のPIDControllerクラスをここに配置"""
    def __init__(self, Kp, Ki, Kd, output_min=None, output_max=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.output_min, self.output_max = output_min, output_max
        self._integral = 0.0
        self._prev_error = None

    def compute(self, error, dt):
        P = self.Kp * error
        self._integral += error * dt
        I = self.Ki * self._integral
        D = 0.0 if self._prev_error is None else self.Kd * (error - self._prev_error) / dt
        self._prev_error = error
        
        output = P + I + D
        # アンチワインドアップと飽和処理
        if self.output_max is not None and output > self.output_max:
            self._integral -= error * dt
            output = self.output_max
        if self.output_min is not None and output < self.output_min:
            self._integral -= error * dt
            output = self.output_min
        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = None