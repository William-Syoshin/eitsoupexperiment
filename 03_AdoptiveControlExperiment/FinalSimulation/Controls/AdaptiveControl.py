import numpy as np

class STRController:
    # 💡 a, T_base の引数を削除。lamのデフォルトを1.0(ノイズに強く)変更
    def __init__(self, alpha_init, beta_init, lam=1.0):
        # 推定パラメータの初期化 [alpha, beta]
        self.theta = np.array([[alpha_init], [beta_init]])
        self.lam = lam  
        
        # 誤差共分散行列の初期化 (シーソー現象のブレを防ぐため1.0)
        self.P = np.eye(2) * 1.0 
        self.alpha_nominal = alpha_init

    # 💡 引数をシンプルに（温度T_measを削除し、補正済みの sigma_comp を直接受け取る）
    def estimate(self, sigma_comp, X_total):
        """RLSアルゴリズムによるパラメータ推定"""
        # 回帰ベクトル phi = [X, 1]^T (2x1の行列)
        phi = np.array([[X_total], [1.0]])
        
        # 予測出力：結果は (1,1) の配列なので .item() で数値に変換
        y_hat = (phi.T @ self.theta).item()
        
        # 推定誤差
        error = sigma_comp - y_hat
        
        # ゲイン行列 K の更新
        P_phi = self.P @ phi
        denominator = self.lam + (phi.T @ P_phi).item()
        K = P_phi / denominator
        
        # パラメータ theta と 誤差共分散行列 P の更新
        self.theta = self.theta + K * error
        self.P = (self.P - (K @ phi.T @ self.P)) / self.lam
        
        return self.theta[0, 0], self.theta[1, 0]

    def get_adjusted_gains(self, Kp_nom, Ki_nom, Kd_nom):
        """推定されたalphaに基づいてPIDゲインを自動調整する"""
        alpha_hat = self.theta[0, 0]
        
        # ゲイン比を最大3倍までに制限（センサー異常時の暴走防止）
        ratio = self.alpha_nominal / max(self.alpha_nominal / 3.0, alpha_hat)
        ratio = min(ratio, 3.0)

        return Kp_nom * ratio, Ki_nom * ratio, Kd_nom * ratio