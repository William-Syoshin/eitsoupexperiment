import numpy as np

class STRController:
    def __init__(self, alpha_init, beta_init, a, T_base, lam=0.98):
        # 推定パラメータの初期化 [alpha, beta]
        self.theta = np.array([[alpha_init], [beta_init]])
        
        # 既知の定数
        self.a = a
        self.T_base = T_base
        self.lam = lam  # 忘却係数 (1に近いほど過去を重視、小さいほど変化に敏感)
        
        # 誤差共分散行列の初期化 (最初は自信がないので大きな値)
        self.P = np.eye(2) * 100.0
        
        # 公称値（ゲイン調整の基準用）
        self.alpha_nominal = alpha_init

    def estimate(self, sigma_meas, C_total, T_meas):
        """RLSアルゴリズムによるパラメータ推定"""
        # 温度補正項の計算
        temp_factor = 1.0 + self.a * (T_meas - self.T_base)
        
        # 回帰ベクトル phi = [C * temp_factor, temp_factor]^T (2x1の行列)
        phi = np.array([[C_total * temp_factor], [temp_factor]])
        
        # 予測出力：結果は (1,1) の配列なので .item() で数値(スカラー)に変換
        y_hat = (phi.T @ self.theta).item()
        
        # 推定誤差
        error = sigma_meas - y_hat
        
        # ゲイン行列 K の更新
        P_phi = self.P @ phi
        # 分母も (1,1) 配列になるため .item() で数値にする
        denominator = self.lam + (phi.T @ P_phi).item()
        K = P_phi / denominator
        
        # パラメータ theta の更新 (2x1)
        self.theta = self.theta + K * error
        
        # 誤差共分散行列 P の更新 (2x2)
        self.P = (self.P - (K @ phi.T @ self.P)) / self.lam
        
        # 推定されたalphaとbetaを返す
        return self.theta[0, 0], self.theta[1, 0]

    def get_adjusted_gains(self, Kp_nom, Ki_nom, Kd_nom):
        """推定されたalphaに基づいてPIDゲインを自動調整する"""
        alpha_hat = self.theta[0, 0]
        
        # alphaが小さくなったら(感度が落ちたら)、その分ゲインを上げて補償する
        # ゲイン比を最大3倍までに制限（センサー異常時の暴走防止）
        ratio = self.alpha_nominal / max(self.alpha_nominal / 3.0, alpha_hat)
        ratio = min(ratio, 3.0)

        return Kp_nom * ratio, Ki_nom * ratio, Kd_nom * ratio