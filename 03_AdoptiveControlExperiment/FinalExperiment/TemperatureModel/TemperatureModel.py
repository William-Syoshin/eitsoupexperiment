import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# ==========================================
# 1. 各物質の比熱（J/g·K）の設定
# ==========================================
# 温まりにくさの固有値です。ここを手打ちで自由に調整できます。
C_WATER  = 4.18  # 水の比熱
C_MISO   = 2.50  # 味噌（ペースト）の比熱の目安
C_POTATO = 3.60  # ポテトの比熱
C_TOFU   = 3.90  # 豆腐（大豆）の比熱

# ==========================================
# 2. 実験ごとの質量（g）の設定（ここを自由に変えられます！）
# ==========================================
# 【実験①：水のみ】
W_MASS_STEP1 = 600.0  # 水の重さ

# 【実験②：水＋ポテト】
W_MASS_STEP2 = 500.0  # 水の重さ
P_MASS_STEP2 = 100.0  # ポテトの重さ

# 【実験③：味噌汁＋豆腐（大豆）】
W_MASS_STEP3 = 465.0  # 水（出汁）の重さ
M_MASS_STEP3 = 35.0   # 溶かした味噌の重さ（例：約7%濃度として35g）
T_MASS_STEP3 = 100.0  # 豆腐（大豆）の重さ


# ==========================================
# 3. 実験データの入力（Sheet2のデータをここに貼る）
# ==========================================
time_data = np.array([0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300])
dt = time_data[1] - time_data[0]
T_room = 25.0  # 室温（初期温度）

# ① 水のみの時の温度データ
Tw_water_data = np.array([24.0, 30.0, 39.2, 43.2, 49.2, 51.2, 53, 53, 53.2, 53.2, 53]) 

# ② 水＋芋の時の温度データ
Tw_potato_data = np.array([25.5, 30, 38, 44, 47.1, 49, 50.7, 50, 49.9, 49.6, 49]) 

# ③ 味噌スープ＋豆腐の時の温度データ（ダミーデータ）
Tw_miso_data = np.array([24.5, 29.0, 36.5, 42.1, 45.0, 46.8, 48.0, 47.8, 47.5, 47.2, 47.0])


# R^2（決定係数）を計算するための共通関数
def calculate_r_squared(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
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

res1 = minimize(objective_step1, [1.0, 0.01], bounds=[(0.001, 10.0), (0.0001, 0.5)])
Qin_opt, k_env_opt = res1.x
r2_water = calculate_r_squared(Tw_water_data, simulate_water_only([Qin_opt, k_env_opt]))


# ==========================================
# Step 2: 「水＋ポテト」のデータから k_loss を特定する
# ==========================================
def simulate_water_potato(k_loss):
    Tw_sim = np.zeros(len(time_data))
    Tp_sim = np.zeros(len(time_data))
    Tw_sim[0] = Tw_potato_data[0]
    Tp_sim[0] = Tw_potato_data[0]
    
    # ★「水」と「ポテト」それぞれの設定値から熱容量比を自動計算！
    C_liquid = W_MASS_STEP2 * C_WATER
    C_solid  = P_MASS_STEP2 * C_POTATO
    capacity_ratio = C_liquid / C_solid
    k_absorb = capacity_ratio * k_loss 
    
    for i in range(1, len(time_data)):
        dTw = (Qin_opt - k_env_opt * (Tw_sim[i-1] - T_room) - k_loss * (Tw_sim[i-1] - Tp_sim[i-1])) * dt
        dTp = (k_absorb * (Tw_sim[i-1] - Tp_sim[i-1])) * dt
        Tw_sim[i] = Tw_sim[i-1] + dTw
        Tp_sim[i] = Tp_sim[i-1] + dTp
    return Tw_sim, Tp_sim

def objective_step2(params):
    Tw_sim, _ = simulate_water_potato(params[0])
    return np.sum((Tw_potato_data - Tw_sim)**2)

res2 = minimize(objective_step2, [0.01], bounds=[(0.0001, 0.5)])
k_loss_potato_opt = res2.x[0]
Tw_pot_sim, Tp_pot_sim = simulate_water_potato(k_loss_potato_opt)
r2_potato = calculate_r_squared(Tw_potato_data, Tw_pot_sim)


# ==========================================
# Step 3: 「味噌汁（豆腐入り）」のデータから k_loss を特定する
# ==========================================
def simulate_miso_soup(k_loss):
    Tw_sim = np.zeros(len(time_data))
    Tp_sim = np.zeros(len(time_data))  # 豆腐の温度
    Tw_sim[0] = Tw_miso_data[0]
    Tp_sim[0] = Tw_miso_data[0]
    
    # ★物理的なポイント：液体側は「水＋味噌」の合算熱容量、固形物は「豆腐」！
    C_liquid = (W_MASS_STEP3 * C_WATER) + (M_MASS_STEP3 * C_MISO)
    C_solid  = T_MASS_STEP3 * C_TOFU
    capacity_ratio = C_liquid / C_solid
    k_absorb = capacity_ratio * k_loss 
    
    for i in range(1, len(time_data)):
        dTw = (Qin_opt - k_env_opt * (Tw_sim[i-1] - T_room) - k_loss * (Tw_sim[i-1] - Tp_sim[i-1])) * dt
        dTp = (k_absorb * (Tw_sim[i-1] - Tp_sim[i-1])) * dt
        Tw_sim[i] = Tw_sim[i-1] + dTw
        Tp_sim[i] = Tp_sim[i-1] + dTp
    return Tw_sim, Tp_sim

def objective_step3(params):
    Tw_sim, _ = simulate_miso_soup(params[0])
    return np.sum((Tw_miso_data - Tw_sim)**2)

res3 = minimize(objective_step3, [0.01], bounds=[(0.0001, 0.5)])
k_loss_miso_opt = res3.x[0]
Tw_miso_sim, Tp_miso_sim = simulate_miso_soup(k_loss_miso_opt)
r2_miso = calculate_r_squared(Tw_miso_data, Tw_miso_sim)

# ==========================================
# PRINT RESULTS (English Version with k_absorb)
# ==========================================
print("===================================================")
print("=== Step 1: Basic Thermal Dynamics (Water Only) ===")
print("===================================================")
print(f"Q_in     (Heating Power MV)        = {Qin_opt:.4f}")
print(f"k_env    (Heat Loss to Ambient)     = {k_env_opt:.4f}")
print(f"★ R^2   (Water Only Model Fit)     = {r2_water:.4f}\n")

print("===================================================")
print("=== Step 2: Potato Dynamics (Water + Potato)    ===")
print("===================================================")
# 熱容量比とk_absorbの計算
ratio_potato = (W_MASS_STEP2 * C_WATER) / (P_MASS_STEP2 * C_POTATO)
k_absorb_potato = ratio_potato * k_loss_potato_opt

print(f"Capacity Ratio (Liquid / Solid)    = {ratio_potato:.2f}")
print(f"k_loss   (Heat transfer: W -> P)   = {k_loss_potato_opt:.4f}")
print(f"k_absorb (Heat absorb: Potato)     = {k_absorb_potato:.4f}") # ★追加
print(f"★ R^2   (Potato Model Fit)         = {r2_potato:.4f}\n")

print("===================================================")
print("=== Step 3: Tofu Dynamics (Miso Soup + Tofu)    ===")
print("===================================================")
# 熱容量比とk_absorbの計算
ratio_miso = ((W_MASS_STEP3 * C_WATER) + (M_MASS_STEP3 * C_MISO)) / (T_MASS_STEP3 * C_TOFU)
k_absorb_miso = ratio_miso * k_loss_miso_opt

print(f"Capacity Ratio (Liquid / Solid)    = {ratio_miso:.2f}")
print(f"k_loss   (Heat transfer: L -> T)   = {k_loss_miso_opt:.4f}")
print(f"k_absorb (Heat absorb: Tofu)       = {k_absorb_miso:.4f}") # ★追加
print(f"★ R^2   (Miso Tofu Model Fit)      = {r2_miso:.4f}")
print("===================================================")


# ==========================================
# グラフ描画
# ==========================================
plt.figure(figsize=(12, 6))

# ① 水のみ
plt.plot(time_data, Tw_water_data, 'bo', label='Data: Water Only')
plt.plot(time_data, simulate_water_only([Qin_opt, k_env_opt]), 'b-', label=f'Model: Water Only ($R^2$={r2_water:.3f})')

# ② 水＋ポテト
plt.plot(time_data, Tw_potato_data, 'ro', label='Data: Water + Potato')
plt.plot(time_data, Tw_pot_sim, 'r-', label=f'Model: Potato Soup Temp ($R^2$={r2_potato:.3f})')
plt.plot(time_data, Tp_pot_sim, 'r--', alpha=0.6, label='Model: Hidden Potato Temp')

# ③ 味噌＋豆腐（大豆）
plt.plot(time_data, Tw_miso_data, 'go', label='Data: Miso + Tofu')
plt.plot(time_data, Tw_miso_sim, 'g-', label=f'Model: Miso Soup Temp ($R^2$={r2_miso:.3f})')
plt.plot(time_data, Tp_miso_sim, 'g--', alpha=0.6, label='Model: Hidden Tofu Temp')

plt.title('Multi-Stage System Identification of Various Soups')
plt.xlabel('Time (s)')
plt.ylabel('Temperature (C)')
plt.legend()
plt.grid(True)
plt.show()