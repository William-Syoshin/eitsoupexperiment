import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. あなたの「真実の係数」と設定
# ==========================================
Qin = 0.2932; k_env = 0.0096; k_loss = 0.0017; k_absorb = 0.0102
T_room = 25.5; Initial_T = 25.5
a_fit = 8.83; B_fit = 0.499

# 目標設定 (全体600g、塩6.0gで1.0%)
target_S_g = 6.0 
T_final_expected = 50.7
target_C = (a_fit * target_S_g + B_fit) * (1.0 + 0.02 * (T_final_expected - 25.5))

# 実測温度データ
time_real = np.array([0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300])
Tw_real = np.array([25.5, 30.0, 38.0, 44.0, 47.1, 49.0, 50.7, 50.0, 49.9, 49.6, 49.0])

# ==========================================
# 2. シミュレーション設定
# ==========================================
dt = 1.0; time_steps = 400
times = np.arange(time_steps) * dt

# --- ① オープンループ ---
S_added_open = np.ones(time_steps) * 6.0
S_liquid_open = np.zeros(time_steps)
Tw_o = Initial_T; Tp_o = Initial_T
Tw_open_arr = np.zeros(time_steps); C_open_arr = np.zeros(time_steps)

# --- ② 普通のPID (30秒ステップ) ---
S_added_pid = np.zeros(time_steps)
S_liquid_pid = np.zeros(time_steps)
Tw_p = Initial_T; Tp_p = Initial_T
Tw_pid_arr = np.zeros(time_steps); C_pid_arr = np.zeros(time_steps)

curr_S_added_pid = 0.0
curr_S_liquid_pid = 0.0
curr_S_liquid_open = 6.0
error_sum = 0.0; Kp = 0.5; Ki = 0.02

# ==========================================
# 3. メインループ (物理演算)
# ==========================================
for i in range(time_steps):
    # 【外乱】ポテトの吸塩 (150秒から徐々に塩が奪われる)
    absorption = 0.005 if 150 < i < 350 else 0.0
    curr_S_liquid_open -= absorption
    curr_S_liquid_pid -= absorption if curr_S_liquid_pid > 0 else 0

    # --- オープンループ ---
    Tw_open_arr[i] = Tw_o
    S_liquid_open[i] = curr_S_liquid_open
    C_open_arr[i] = (a_fit * curr_S_liquid_open + B_fit) * (1.0 + 0.02 * (Tw_o - 25.5))
    Tw_o += (Qin - k_env*(Tw_o - T_room) - k_loss*(Tw_o - Tp_o)) * dt
    Tp_o += (k_absorb*(Tw_o - Tp_o)) * dt

    # --- PID (30秒に1回判断) ---
    Tw_pid_arr[i] = Tw_p
    current_C = (a_fit * curr_S_liquid_pid + B_fit) * (1.0 + 0.02 * (Tw_p - 25.5))
    C_pid_arr[i] = current_C
    S_added_pid[i] = curr_S_added_pid
    S_liquid_pid[i] = curr_S_liquid_pid

    if i % 30 == 0:
        error = target_C - current_C
        error_sum += error * 30
        u_salt = Kp * error + Ki * error_sum
        if u_salt > 0.5: u_salt = 0.5
        elif u_salt < 0: u_salt = 0.0
        curr_S_added_pid += u_salt
        curr_S_liquid_pid += u_salt

    Tw_p += (Qin - k_env*(Tw_p - T_room) - k_loss*(Tw_p - Tp_p)) * dt
    Tp_p += (k_absorb*(Tw_p - Tp_p)) * dt

# ==========================================
# 4. 2x2 レイアウトでの描画
# ==========================================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
axes = axes.flatten() # 2次元配列を1次元にして扱いやすくする

# 左上: 投入した塩の総量 (Robot Action)
axes[0].plot(times, S_added_open, 'b--', label='Open Loop (6g Fixed)')
axes[0].step(times, S_added_pid, 'r-', where='post', label='Normal PID (30s Step)')
axes[0].axhline(y=6.0, color='gray', linestyle=':')
axes[0].set_title('1. Total Salt Added (g)')
axes[0].set_ylabel('Grams (g)')
axes[0].legend(); axes[0].grid(True)

# 右上: スープの実際の塩分濃度 (Taste)
# 濃度(%) = (液体中の塩の量 / 全体量600g) * 100
axes[1].plot(times, (S_liquid_open / 600.0) * 100, 'b--', label='Open Loop')
axes[1].plot(times, (S_liquid_pid / 600.0) * 100, 'r-', label='Normal PID')
axes[1].axhline(y=1.0, color='k', linestyle=':', label='Target 1.0%')
axes[1].set_title('2. Liquid Salinity Concentration (%)')
axes[1].set_ylabel('Concentration (%)')
axes[1].legend(); axes[1].grid(True)

# 左下: 伝導率 (Sensor Feedback)
axes[2].plot(times, C_open_arr, 'b--', label='Open Loop')
axes[2].plot(times, C_pid_arr, 'r-', label='Normal PID')
axes[2].axhline(y=target_C, color='r', linestyle=':', label=f'Target C ({target_C:.2f})')
axes[2].set_title('3. Conductivity (mS/cm)')
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('mS/cm')
axes[2].legend(); axes[2].grid(True)

# 右下: 温度 (Physical Model)
axes[3].plot(time_real, Tw_real, 'ko', label='Real Data (Water)')
axes[3].plot(times, Tw_open_arr, 'b-', label='Thermal Model')
axes[3].set_title('4. Temperature (C)')
axes[3].set_xlabel('Time (s)')
axes[3].set_ylabel('Celsius (C)')
axes[3].legend(); axes[3].grid(True)

plt.tight_layout()
plt.show()