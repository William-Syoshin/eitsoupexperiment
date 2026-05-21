class OpenLoopController:
    """
    オープンループ制御器
    指定されたタイミングで固定量を投入する、または常に0を返す
    """
    def __init__(self, fixed_rate=0.0):
        self.fixed_rate = fixed_rate

    def compute(self, error, dt):
        # 偏差（error）を無視して固定値を返す
        return self.fixed_rate

    def reset(self):
        pass