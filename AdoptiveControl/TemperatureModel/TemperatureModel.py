import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# ==========================================
# 1. 実験データの入力（Sheet2のデータをここに貼る）
# ==========================================
# 時間 (秒) ※例として30秒刻みで入れています
time_data = np.array([0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300])

# ① 水のみの時の温度データ
Tw_water_data = np.array([24.0, 30.0, 39.2, 43.2, 49.2, 51.2, 53, 53, 53.2, 53.2, 53]) 

# ② 水＋芋の時の温度データ（水のみより少し温度が低いはず）
Tw_potato_data = np.array([25.5, 30, 38, 44, 47.1, 49, 50.7, 50, 49.9, 49.6, 49]) 

dt = time_data[1] - time_data[0]
T_room = 25.0 # 室温（初期温度）
# ★追加：R^2を計算するための共通関数
def calculate_r_squared(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred) ** 2)          # 残差変動
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2) # 全変動
    return 1 - (ss_res / ss_tot)
# ==========================================
# Step 1: 「水のみ」のデータから Qin と k_env を特定する
# ==========================================
def simulate_water_only(params):
    Qin, k_env = params
    Tw_sim = np.zeros(len(time_data))
    Tw_sim[0] = Tw_water_data[0]
    
    for i in range(1, len(time_data)):
        dTw = (Qin - k_env * (Tw_sim[i-1] - T_room)) * dt
        Tw_sim[i] = Tw_sim[i-1] + dTw
    return Tw_sim

def objective_step1(params):
    Tw_sim = simulate_water_only(params)
    return np.sum((Tw_water_data - Tw_sim)**2)

# 最適化実行 (Qinとk_envを探す)
res1 = minimize(objective_step1, [1.0, 0.01], bounds=[(0.001, 10.0), (0.0001, 0.5)])
Qin_opt, k_env_opt = res1.x

# ★Step 1のR^2を計算
Tw_water_sim_best = simulate_water_only([Qin_opt, k_env_opt])
r2_water = calculate_r_squared(Tw_water_data, Tw_water_sim_best)

print("=== Step 1: IHと鍋の基本特性（水のみデータより） ===")
print(f"Q_in  (一定加熱力) = {Qin_opt:.4f}")
print(f"k_env (空気への放熱) = {k_env_opt:.4f}")
print(f"★ R^2 (水のみモデル) = {r2_water:.4f}")

# ==========================================
# Step 2: 「水＋芋」のデータから k_loss を特定する
# ==========================================
# ※ Step 1で求めた Qin と k_env は固定して使います
def simulate_water_potato(k_loss):
    Tw_sim = np.zeros(len(time_data))
    Tp_sim = np.zeros(len(time_data))
    Tw_sim[0] = Tw_potato_data[0]
    Tp_sim[0] = Tw_potato_data[0]
    
    # 物理法則に基づく熱容量比 (水500g : 芋100g = 1 : 6)
    k_absorb = 6.0 * k_loss 
    
    for i in range(1, len(time_data)):
        # モデル式（Qinとk_envはStep1の結果を使用）
        dTw = (Qin_opt - k_env_opt * (Tw_sim[i-1] - T_room) - k_loss * (Tw_sim[i-1] - Tp_sim[i-1])) * dt
        dTp = (k_absorb * (Tw_sim[i-1] - Tp_sim[i-1])) * dt
        
        Tw_sim[i] = Tw_sim[i-1] + dTw
        Tp_sim[i] = Tp_sim[i-1] + dTp
        
    return Tw_sim, Tp_sim

def objective_step2(params):
    k_loss = params[0]
    Tw_sim, _ = simulate_water_potato(k_loss)
    return np.sum((Tw_potato_data - Tw_sim)**2)

# 最適化実行 (k_lossを探す)
res2 = minimize(objective_step2, [0.01], bounds=[(0.0001, 0.5)])
k_loss_opt = res2.x[0]
# ★Step 2のR^2を計算
Tw_potato_sim_best, Tp_potato_sim_best = simulate_water_potato(k_loss_opt)
r2_potato = calculate_r_squared(Tw_potato_data, Tw_potato_sim_best)

print("\n=== Step 2: 芋の熱吸収特性（水＋芋データより） ===")
print(f"k_loss   (水から芋へ逃げる熱) = {k_loss_opt:.4f}")
print(f"k_absorb (芋が温まるスピード) = {k_loss_opt * 6.0:.4f} (※物理法則より自動計算)")
print(f"★ R^2 (水＋芋モデル)  = {r2_potato:.4f}")
print("===================================================\n")

# ==========================================
# 結果のグラフ描画
# ==========================================
Tw_water_sim = simulate_water_only([Qin_opt, k_env_opt])
Tw_potato_sim, Tp_potato_sim = simulate_water_potato(k_loss_opt)

plt.figure(figsize=(12, 6))

# 水のみのグラフ（R^2を凡例に追加）
plt.plot(time_data, Tw_water_data, 'bo', label='Data: Water Only')
plt.plot(time_data, Tw_water_sim_best, 'b-', label=f'Model: Water Only ($R^2$={r2_water:.3f})')

# 水＋芋のグラフ（R^2を凡例に追加）
plt.plot(time_data, Tw_potato_data, 'ro', label='Data: Water + Potato')
plt.plot(time_data, Tw_potato_sim_best, 'r-', label=f'Model: Water Temp ($R^2$={r2_potato:.3f})')
plt.plot(time_data, Tp_potato_sim_best, 'r--', label='Model: Hidden Potato Temp')

plt.title('2-Stage System Identification of Thermal Dynamics')
plt.xlabel('Time (s)')
plt.ylabel('Temperature (C)')
plt.legend()
plt.grid(True)
plt.show()
plt.show()