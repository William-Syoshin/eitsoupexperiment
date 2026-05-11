import numpy as np
import matplotlib.pyplot as plt
 
# ===== パラメータ =====
C_target = 1.75   # 目標塩分濃度（%）
T_target = 60.0   # 目標温度（℃）
T_inf = 25.0      # 環境温度（℃）
water_mass = 200.0  # 水の質量（g）
Tc = 1.0          # サイクル時間（秒）
 
# 目標Vを計算
def C_to_sigma(C, T):
    return 32.21*C + 0.71*T - 0.28*C*T - 32.40
 
def sigma_to_V(sigma):
    return 0.7430 / sigma + 0.0333
 
def V_to_sigma(V):
    return 0.7430 / (V - 0.0333)
 
def sigma_to_C(sigma, T):
    return (sigma + 32.40 - 0.71*T) / (32.21 - 0.28*T)
 
# 目標V（60℃・1.75%のとき）
sigma_target = C_to_sigma(C_target, T_target)
V_target = sigma_to_V(sigma_target)
print(f"sigma_target = {sigma_target:.3f} mS/cm")
print(f"V_target = {V_target:.4f} V")
 
# ===== PIDゲイン =====
# 温度制御（u_Q）
kp_Q_T = 5.0    # 温度誤差に対する熱入力
kp_Q_V = 2.0    # V誤差に対する熱入力（温度の影響）
 
# 塩制御（u_salt）
kp_s_V = 0.3    # V誤差に対する塩入力
 
ki_Q_T = 0.1
ki_s_V = 0.05
 
kd_Q_T = 0.5
kd_s_V = 0.05
 
# ===== 温度モデル =====
def temperature_model(T_current, u_Q, T_inf=25):
    # 熱入力と放熱のバランス
    heat_loss = 0.05 * (T_current - T_inf)
    dT = (u_Q - heat_loss) * 0.1
    T_new = T_current + dT
    return min(T_new, 95)  # 沸騰しない
 
# ===== 塩モデル =====
def add_salt(total_salt, u_salt, water_mass):
    total_salt_new = total_salt + max(0, u_salt)
    C_new = total_salt_new / (water_mass + total_salt_new) * 100
    return C_new, total_salt_new
 
# ===== シミュレーション =====
T_current = 25.0
C_current = 0.0
total_salt = 0.0
 
eT_prev = 0
eV_prev = 0
eT_integral = 0
eV_integral = 0
 
# 記録用
time_log = []
T_log = []
C_log = []
V_log = []
eT_log = []
eV_log = []
uQ_log = []
us_log = []
 
for n in range(200):
 
    # ①センサー測定（シミュレーション）
    sigma = C_to_sigma(C_current, T_current)
    V = sigma_to_V(sigma)
 
    # ②誤差計算
    eT = T_target - T_current
    eV = V_target - V
 
    # ③PID制御
 
    # 熱入力（温度誤差とV誤差の両方を見る）
    eT_integral += eT * Tc
    u_Q = (kp_Q_T * eT
         + kp_Q_V * eV
         + ki_Q_T * eT_integral
         + kd_Q_T * (eT - eT_prev)/Tc)
    u_Q = max(0, u_Q)  # 熱は取り除けない
 
    # 塩入力（V誤差を見る）
    eV_integral += eV * Tc
    u_salt = (kp_s_V * eV
            + ki_s_V * eV_integral
            + kd_s_V * (eV - eV_prev)/Tc)
    u_salt = max(0, u_salt)  # 塩は取り出せない
 
    # ④状態更新
    T_current = temperature_model(T_current, u_Q)
    C_current, total_salt = add_salt(total_salt, u_salt, water_mass)
 
    # ⑤記録
    time_log.append(n * Tc)
    T_log.append(T_current)
    C_log.append(C_current)
    V_log.append(V)
    eT_log.append(eT)
    eV_log.append(eV)
    uQ_log.append(u_Q)
    us_log.append(u_salt)
 
    eT_prev = eT
    eV_prev = eV
 
    # 収束判定
    if abs(eT) < 0.5 and abs(eV) < 0.001:
        print(f"Converged at cycle {n}")
        break
 
# ===== グラフ =====
fig, axes = plt.subplots(3, 2, figsize=(12, 12))
fig.suptitle('PID Simulation: Temperature + EIT Voltage Control\n'
             f'Target T={T_target}C, Target C={C_target}%, Water=200ml',
             fontsize=13)
 
# 左上：温度
axes[0,0].plot(time_log, T_log, color='red', linewidth=2, label='T (actual)')
axes[0,0].axhline(T_target, linestyle='--', color='blue', label=f'Target {T_target}C')
axes[0,0].axhline(25, linestyle=':', color='gray', label='Start 25C')
axes[0,0].set_ylabel('Temperature (C)')
axes[0,0].set_xlabel('Time (s)')
axes[0,0].set_title('Temperature Tracking')
axes[0,0].legend()
axes[0,0].grid(True, alpha=0.3)
 
# 右上：EIT電圧
axes[0,1].plot(time_log, V_log, color='purple', linewidth=2, label='V (actual)')
axes[0,1].axhline(V_target, linestyle='--', color='blue', label=f'Target V={V_target:.4f}')
axes[0,1].set_ylabel('EIT Voltage (V)')
axes[0,1].set_xlabel('Time (s)')
axes[0,1].set_title('EIT Voltage Tracking')
axes[0,1].legend()
axes[0,1].grid(True, alpha=0.3)
 
# 左中：塩分濃度
axes[1,0].plot(time_log, C_log, color='orange', linewidth=2, label='C (actual)')
axes[1,0].axhline(C_target, linestyle='--', color='blue', label=f'Target {C_target}%')
axes[1,0].axhline(1.5, linestyle=':', color='gray', label='1.5%')
axes[1,0].axhline(2.0, linestyle=':', color='gray', label='2.0%')
axes[1,0].set_ylabel('Salt Concentration (%)')
axes[1,0].set_xlabel('Time (s)')
axes[1,0].set_title('Salt Concentration')
axes[1,0].legend()
axes[1,0].grid(True, alpha=0.3)
 
# 右中：誤差
axes[1,1].plot(time_log, eT_log, color='red', linewidth=2, label='Temperature error')
ax2 = axes[1,1].twinx()
ax2.plot(time_log, eV_log, color='purple', linewidth=2, 
         linestyle='--', label='V error')
axes[1,1].set_ylabel('Temperature Error (C)', color='red')
ax2.set_ylabel('V Error (V)', color='purple')
axes[1,1].set_xlabel('Time (s)')
axes[1,1].set_title('Errors')
axes[1,1].grid(True, alpha=0.3)
 
# 左下：熱入力
axes[2,0].plot(time_log, uQ_log, color='red', linewidth=2)
axes[2,0].set_ylabel('Heat Input u_Q (W)')
axes[2,0].set_xlabel('Time (s)')
axes[2,0].set_title('Heat Input')
axes[2,0].grid(True, alpha=0.3)
 
# 右下：塩入力
axes[2,1].bar(time_log, us_log, color='green', alpha=0.7)
axes[2,1].set_ylabel('Salt Input u_salt (g)')
axes[2,1].set_xlabel('Time (s)')
axes[2,1].set_title(f'Salt Input (Total: {total_salt:.3f}g)')
axes[2,1].grid(True, alpha=0.3)
 
plt.tight_layout()
plt.savefig('pid_simulation_v2.png', 
            dpi=150, bbox_inches='tight')
plt.show()  # ← これを追加すると画面にも表示される

print("Graph saved!")
 
print(f"\n===== Final Results =====")
print(f"Target temperature:  {T_target}C")
print(f"Final temperature:   {T_current:.2f}C")
print(f"Target V:            {V_target:.4f} V")
print(f"Final V:             {V_log[-1]:.4f} V")
print(f"Target C:            {C_target}%")
print(f"Final C:             {C_current:.4f}%")
print(f"Total salt added:    {total_salt:.4f}g")
 