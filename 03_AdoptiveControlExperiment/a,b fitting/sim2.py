import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. データの入力とフィッティング (a と B を求める)
# ==========================================
# 画像から読み取った実測値 (50℃)
x_data = np.array([0, 1/6, 2/6, 3/6, 4/6, 5/6, 6/6])
y_data = np.array([0.25, 3.64, 6.11, 10.1, 12.82, 14.95, 18.3])

# 最小二乗法で50℃の傾きと切片を出す
a_50, b_50 = np.polyfit(x_data, y_data, 1)

# 25℃基準の係数に変換 (1.5で割る)
a_fit = a_50 / 1.5
b_fit = b_50 / 1.5

print(f"決定した係数: a={a_fit:.4f}, b={b_fit:.4f}")

# ==========================================
# 2. シミュレーションの設定
# ==========================================
Qin = 0.2932; k_env = 0.0096; k_loss = 0.0017; k_absorb = 0.0102
T_room = 25.5; target_S = 6.0; total_weight = 600.0
target_C = (a_fit * target_S + b_fit) * (1.0 + 0.02 * (50.7 - 25.5))

dt = 1.0; steps = 400; times = np.arange(steps)
S_added = np.zeros(steps); S_liquid = np.zeros(steps)
Tw_arr = np.zeros(steps); C_arr = np.zeros(steps)

# 実測温度
t_real = np.array([0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300])
T_real = np.array([25.5, 30.0, 38.0, 44.0, 47.1, 49.0, 50.7, 50.0, 49.9, 49.6, 49.0])

# ==========================================
# 3. メインループ (30秒ごとのPID)
# ==========================================
Tw = 25.5; Tp = 25.5; S_total = 0.0; S_liq = 0.0; err_sum = 0.0

for i in range(steps):
    # 外乱：ポテトの吸塩 (150秒〜)
    if 150 < i < 350: S_liq -= 0.005 
    
    # 温度と伝導率の計算
    Tw_arr[i] = Tw
    C = (a_fit * S_liq + b_fit) * (1.0 + 0.02 * (Tw - 25.5))
    C_arr[i] = C
    S_added[i] = S_total
    S_liquid[i] = S_liq

    # 30秒ごとのPID
    if i % 30 == 0:
        err = target_C - C
        err_sum += err * 30
        u = 0.6 * err + 0.02 * err_sum
        u = np.clip(u, 0, 0.5) # 0.5g制限
        S_total += u; S_liq += u

    # 物理更新
    Tw += (Qin - k_env*(Tw-T_room) - k_loss*(Tw-Tp)) * dt
    Tp += (k_absorb*(Tw-Tp)) * dt

# ==========================================
# 4. 2x2 グラフ表示
# ==========================================
fig, ax = plt.subplots(2, 2, figsize=(12, 8))

# 投入量
ax[0,0].step(times, S_added, 'r'); ax[0,0].set_title("1. Total Salt Added (g)")
ax[0,0].axhline(6, color='k', ls=':'); ax[0,0].grid(True)

# 濃度 (%)
ax[0,1].plot(times, (S_liquid/total_weight)*100, 'r')
ax[0,1].axhline(1.0, color='k', ls='-'); ax[0,1].set_title("2. Salinity (%)")
ax[0,1].grid(True); ax[0,1].set_ylim(0, 1.2)

# 伝導率
ax[1,0].plot(times, C_arr, 'r'); ax[1,0].set_title("3. Conductivity (mS/cm)")
ax[1,0].axhline(target_C, color='k', ls=':'); ax[1,0].grid(True)

# 温度
ax[1,1].plot(t_real, T_real, 'ko'); ax[1,1].plot(times, Tw_arr, 'r')
ax[1,1].set_title("4. Temperature (C)"); ax[1,1].grid(True)

plt.tight_layout(); plt.show()