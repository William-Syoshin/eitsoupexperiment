import numpy as np
import matplotlib.pyplot as plt

# ===== パラメータ =====
C_target = 1.75
T_target = 60.0
T_start = 25.0
water_mass = 200.0
Tc = 10.0          # 10秒ごとに制御
n_cycles = 30      # 30サイクル = 300秒

dT_per_sec = (T_target - T_start) / 300  # 300秒で60℃

# ===== 方程式 =====
def C_to_sigma(C, T):
    sigma = 32.21*C + 0.71*T - 0.28*C*T - 32.40
    return max(sigma, 1.0)  # マイナス防止

def sigma_to_V(sigma):
    return 0.7430 / sigma + 0.0333

def V_to_sigma(V):
    return 0.7430 / (V - 0.0333)

def sigma_to_C(sigma, T):
    return (sigma + 32.40 - 0.71*T) / (32.21 - 0.28*T)

print(f"サイクル数 = {n_cycles}回（{n_cycles * Tc:.0f}秒）")

# ===== PIDゲイン =====
# 誤差はCで計算するのでゲインを調整
kp = 0.25
ki = 0.0004
kd = 0.5
max_salt_per_cycle = 0.5  # g

# ===== 塩モデル =====
def add_salt(total_salt, u_salt, water_mass):
    total_salt_new = total_salt + max(0, u_salt)
    C_new = total_salt_new / (water_mass + total_salt_new) * 100
    return C_new, total_salt_new

# ===== シミュレーション =====
T_current = T_start
C_current = 0.0
total_salt = 0.0

e_prev = 0
e_integral = 0

time_log = []
T_log = []
C_log = []
C_estimated_log = []
V_log = []
e_log = []
us_log = []

for n in range(n_cycles):

    # ①時刻と温度
    t_now = n * Tc
    T_current = T_start + dT_per_sec * t_now
    T_current = min(T_current, T_target)

    # ②EITシミュレーション
    # 実際のσを計算してVに変換（センサーの代わり）
    sigma_real = C_to_sigma(C_current, T_current)
    V_measured = sigma_to_V(sigma_real)

    # ③VとTからCを推定（実際のEIT処理）
    sigma_estimated = V_to_sigma(V_measured)
    C_estimated = sigma_to_C(sigma_estimated, T_current)

    # ④誤差計算（Cで比較）
    e = C_target - C_estimated

    # ⑤PID制御
    e_integral += e * Tc
    u_salt = (kp * e
            + ki * e_integral
            + kd * (e - e_prev) / Tc)
    u_salt = max(0, u_salt)
    u_salt = min(u_salt, max_salt_per_cycle)

    # ⑥塩を追加
    C_current, total_salt = add_salt(total_salt, u_salt, water_mass)

    # ⑦記録
    time_log.append(t_now)
    T_log.append(T_current)
    C_log.append(C_current)
    C_estimated_log.append(C_estimated)
    V_log.append(V_measured)
    e_log.append(e)
    us_log.append(u_salt)

    e_prev = e

# ===== グラフ =====
fig, axes = plt.subplots(3, 2, figsize=(12, 12))
fig.suptitle('PID Simulation: C Control via EIT Voltage\n'
             f'Fixed heating: {T_start}C -> {T_target}C in 300s\n'
             f'Tc={Tc}s, {n_cycles} cycles, Target C={C_target}%\n'
             f'kp={kp}, ki={ki}, kd={kd}, max={max_salt_per_cycle}g/cycle',
             fontsize=11)

# 左上：温度
axes[0,0].plot(time_log, T_log, color='red', linewidth=2)
axes[0,0].axhline(T_target, linestyle='--', color='blue',
                   label=f'Target {T_target}C')
axes[0,0].set_ylabel('Temperature (C)')
axes[0,0].set_xlabel('Time (s)')
axes[0,0].set_title('Temperature (Fixed Heating)')
axes[0,0].legend()
axes[0,0].grid(True, alpha=0.3)

# 右上：EIT電圧
axes[0,1].plot(time_log, V_log, color='purple', linewidth=2,
               marker='o', markersize=5, label='V measured')
axes[0,1].set_ylabel('EIT Voltage V (V)')
axes[0,1].set_xlabel('Time (s)')
axes[0,1].set_title('EIT Voltage')
axes[0,1].legend()
axes[0,1].grid(True, alpha=0.3)

# 左中：塩分濃度
axes[1,0].plot(time_log, C_log, color='orange', linewidth=2,
               marker='o', markersize=5, label='C actual')
axes[1,0].plot(time_log, C_estimated_log, color='green', linewidth=2,
               linestyle='--', marker='x', markersize=5,
               label='C estimated (from V)')
axes[1,0].axhline(C_target, linestyle='--', color='blue',
                   label=f'Target {C_target}%')
axes[1,0].axhline(1.5, linestyle=':', color='gray', label='1.5%')
axes[1,0].axhline(2.0, linestyle=':', color='gray', label='2.0%')
axes[1,0].set_ylabel('Salt Concentration (%)')
axes[1,0].set_xlabel('Time (s)')
axes[1,0].set_title('Salt Concentration')
axes[1,0].legend()
axes[1,0].grid(True, alpha=0.3)

# 右中：C誤差
axes[1,1].plot(time_log, e_log, color='purple', linewidth=2,
               marker='o', markersize=5)
axes[1,1].axhline(0, linestyle='--', color='black')
axes[1,1].set_ylabel('C Error (%)')
axes[1,1].set_xlabel('Time (s)')
axes[1,1].set_title('Concentration Error')
axes[1,1].grid(True, alpha=0.3)

# 左下：塩入力
axes[2,0].bar(time_log, us_log, color='green', alpha=0.7, width=8)
axes[2,0].axhline(max_salt_per_cycle, linestyle='--',
                   color='red', label=f'Max {max_salt_per_cycle}g')
axes[2,0].set_ylabel('Salt Input per cycle (g)')
axes[2,0].set_xlabel('Time (s)')
axes[2,0].set_title(f'Salt Input (Total: {total_salt:.3f}g)')
axes[2,0].legend()
axes[2,0].grid(True, alpha=0.3)

# 右下：累積塩
cumulative_salt = np.cumsum(us_log)
axes[2,1].plot(time_log, cumulative_salt, color='green',
               linewidth=2, marker='o', markersize=5)
axes[2,1].set_ylabel('Cumulative Salt (g)')
axes[2,1].set_xlabel('Time (s)')
axes[2,1].set_title('Cumulative Salt Added')
axes[2,1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('pid_simulation_v8.png', dpi=150, bbox_inches='tight')
plt.show()
print("Graph saved!")

print(f"\n===== Final Results =====")
print(f"Tc:             {Tc}秒ごとに制御")
print(f"Total cycles:   {n_cycles}回（{n_cycles * Tc:.0f}秒）")
print(f"Final T:        {T_current:.2f} C")
print(f"Final V:        {V_log[-1]:.4f} V")
print(f"Final C actual: {C_current:.4f} %")
print(f"Final C estim:  {C_estimated_log[-1]:.4f} %")
print(f"Target C:       {C_target} %")
print(f"Total salt:     {total_salt:.4f} g")