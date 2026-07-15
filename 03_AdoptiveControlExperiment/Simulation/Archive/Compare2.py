import numpy as np
import matplotlib.pyplot as plt
from Soup_Plant import SoupPlant
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController

def run_simulation(
    soup_type="miso",       
    control_mode="PID",
    salt_interval=30.0,    
    salt_limit=0.5,        
    temp_interval=1.0,
    kp_c=0.1, ki_c=0.005, kd_c=0.0,
    kp_t=0.05, ki_t=0.0005, kd_t=0.0
):
    DT = 1.0; SIM_TIME = 600.0; STEPS = int(SIM_TIME / DT)
    time = np.arange(STEPS) * DT
    T_REF = 50.0; C_REF = 1.0

    # 【ロボットの予習知識】
    ALPHA_NOMINAL = 8.1013
    BETA_NOMINAL  = 0.1550
    TEMP_COEFF    = 0.02
    T_BASE        = 24.6
    KNOWN_BASE_MASS = 600.0

    # 💡 パラメータ調整用の安全装置設定
    ALPHA_MIN = 1 #ALPHA_NOMINAL / 3.0
    ALPHA_MAX = 20 #ALPHA_NOMINAL * 3.0
    BETA_MIN  = -5.0  # X軸シフトによりマイナスになるため広めに設定
    BETA_MAX  = 2.0
    
    # 1ステップあたりの最大変化量（変化率制限）
    D_ALPHA_MAX = 1.0
    D_BETA_MAX  = 0.5
    
    # 不感帯（これ以下の変化はノイズとみなして無視）
    DEADBAND = 0.01 

    plant = SoupPlant(soup_type=soup_type, T_w_init=50.0, T_s_init=50.0, T_room=24.6)
    rate_limit = salt_limit / salt_interval

    if control_mode == "OpenLoop":
        conc_controller = OpenLoopController(fixed_rate=0.0)
    else:
        conc_controller = PIDController(Kp=kp_c, Ki=ki_c, Kd=kd_c, output_min=0.0, output_max=rate_limit)

    pid_temp = PIDController(Kp=kp_t, Ki=ki_t, Kd=kd_t, output_min=0.0)

    str_unit = None
    if control_mode == "STR":
        str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL)

    logs = {key: np.zeros(STEPS) for key in ["Tw", "C", "sigma", "Qin", "salt_cum", "alpha_hat", "beta_hat"]}
    
    robot_added_salt_total = 0.0
    C_init_virtual = 0.0 # 💡 仮想の初期塩分を記憶する変数
    
    # 前回のパラメータ記憶用
    prev_alpha = ALPHA_NOMINAL
    prev_beta = BETA_NOMINAL

    for i in range(STEPS):
        if i % temp_interval == 0:
            e_T = T_REF - plant.T_w
            Q_in = max(0.0, pid_temp.compute(e_T, temp_interval))

        if i % salt_interval == 0 and i > 0:
            temp_comp = 1.0 + TEMP_COEFF * (plant.T_w - T_BASE)
            if temp_comp == 0: temp_comp = 1.0
            sigma_comp = plant.conductivity / temp_comp
            
            if control_mode == "OpenLoop":
                salt_added = 6.0 if i == salt_interval else 0.0
                
            elif control_mode == "PID":
                C_hat_fixed = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                e_C = C_REF - C_hat_fixed
                salt_added = conc_controller.compute(e_C, salt_interval) * salt_interval
                
            elif control_mode == "STR":
                # 💡 1. 最初の1歩目で「仮想の初期塩分」を逆算
                if robot_added_salt_total < 0.01:
                    C_init_virtual = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL

                # 💡 2. X軸を「推測初期塩分＋追加量」の『仮想総濃度』にする
                C_added = (robot_added_salt_total / KNOWN_BASE_MASS) * 100.0
                X_virtual_total = C_init_virtual + C_added
                
                # 3. 生のパラメータを学習
                raw_alpha_h, raw_beta_h = str_unit.estimate(sigma_comp, X_virtual_total)
                
                # 💡 4. 【安全装置】変化率制限と不感帯の適用
                # 変化量を計算
                d_alpha = raw_alpha_h - prev_alpha
                d_beta  = raw_beta_h - prev_beta
                
                # 不感帯（小さすぎる変化は無視）
                if abs(d_alpha) < DEADBAND: d_alpha = 0.0
                if abs(d_beta)  < DEADBAND: d_beta  = 0.0
                
                # 変化率制限（大きすぎる変化をカット）
                d_alpha = max(-D_ALPHA_MAX, min(d_alpha, D_ALPHA_MAX))
                d_beta  = max(-D_BETA_MAX,  min(d_beta,  D_BETA_MAX))
                
                # パラメータの更新と上下限クリップ
                alpha_h = max(ALPHA_MIN, min(prev_alpha + d_alpha, ALPHA_MAX))
                beta_h  = max(BETA_MIN,  min(prev_beta  + d_beta,  BETA_MAX))
                
                # 値を記憶してSTR内部に上書き（次回の学習が暴走しないように）
                prev_alpha = alpha_h
                prev_beta  = beta_h
                str_unit.theta[0, 0] = alpha_h
                str_unit.theta[1, 0] = beta_h
                
                # ゲインの適応調整
                adaptive_ratio = ALPHA_NOMINAL / alpha_h
                adaptive_ratio = min(max(adaptive_ratio, 0.3), 3.0)
                conc_controller.Kp = kp_c * adaptive_ratio
                conc_controller.Ki = ki_c * adaptive_ratio
                conc_controller.Kd = kd_c * adaptive_ratio
                
                # 💡 5. X軸をズラしたので、beta_h を直接使って総濃度を計算！
                if robot_added_salt_total < 0.01:
                    C_true_abs = C_init_virtual
                else:
                    C_true_abs = (sigma_comp - beta_h) / alpha_h
                
                error = C_REF - C_true_abs
                salt_added = conc_controller.compute(error, salt_interval) * salt_interval
        else:
            salt_added = 0.0
       
        plant.step(Q_in=Q_in, salt_added_this_step=salt_added, dt=DT)
        robot_added_salt_total += salt_added
        
        # 記録
        logs["Tw"][i] = plant.T_w
        logs["C"][i] = plant.concentration
        logs["sigma"][i] = plant.conductivity
        logs["Qin"][i] = Q_in
        logs["salt_cum"][i] = robot_added_salt_total 
        if control_mode == "STR":
            # 安全装置を通った後の値を記録
            logs["alpha_hat"][i] = prev_alpha
            logs["beta_hat"][i]  = prev_beta

    return time, logs


# ============================================================
# ⚙️ 描画処理：3つのスープ×6パネル ＋ まとめグラフ
# ============================================================
colors = {'OL': '#00FF00', 'PID': '#0000FF', 'STR': '#FF8C00'}
target_soups = ["water", "miso", "miso_tofu"] 

robustness_errors = {'OL': [], 'PID': [], 'STR': []}
C_REF = 1.0
EVAL_START_TIME = 500.0  

for soup in target_soups:
    print(f"Simulating {soup}...")
    
    t, logs_open = run_simulation(soup_type=soup, control_mode="OpenLoop", kp_c=0.1, ki_c=0.005, kd_c=0.0)
    _, logs_pid  = run_simulation(soup_type=soup, control_mode="PID",      kp_c=0.1, ki_c=0.005, kd_c=0.0)
    _, logs_str  = run_simulation(soup_type=soup, control_mode="STR",      kp_c=0.1, ki_c=0.005, kd_c=0.0)
    
    idx = (t >= EVAL_START_TIME)
    robustness_errors['OL'].append(np.mean(np.abs(logs_open["C"][idx] - C_REF)) / C_REF * 100.0)
    robustness_errors['PID'].append(np.mean(np.abs(logs_pid["C"][idx] - C_REF)) / C_REF * 100.0)
    robustness_errors['STR'].append(np.mean(np.abs(logs_str["C"][idx] - C_REF)) / C_REF * 100.0)
    
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), num=f"Soup Profile: {soup.upper()}")
    fig.suptitle(f"Control Performance Analysis - [{soup.upper()}]", fontsize=14, fontweight="bold")

    # ① Temperature Profile
    ax = axes[0, 0]
    ax.plot(t, logs_open["Tw"], color=colors['OL'], alpha=0.3)
    ax.plot(t, logs_pid["Tw"],  color=colors['PID'], alpha=0.4)
    ax.plot(t, logs_str["Tw"],  color="#FF4500", lw=2)
    ax.axhline(50.0, color="black", ls=":", label="Target (50°C)")
    ax.set(ylabel="Temp [°C]", title="① Temperature Profile")
    ax.legend(loc='lower right')
    ax.grid(alpha=0.3)

    # ② Heating Power (Qin)
    ax = axes[0, 1]
    ax.plot(t, logs_open["Qin"], color="#E0B0FF", alpha=0.5, label='Open-Loop')
    ax.plot(t, logs_pid["Qin"],  color="#9932CC", alpha=0.6, label='PID')
    ax.plot(t, logs_str["Qin"],  color="#4B0082", lw=1.5, label='STR')
    ax.set(ylabel="Q_in [°C/s]", title="② Manipulated Variable: Heating Power")
    ax.legend(loc='upper right')
    ax.grid(alpha=0.3)

    # ③ Salinity Concentration
    ax = axes[1, 0]
    ax.step(t, logs_open["C"], color=colors['OL'], where='post', label='Open-Loop')
    ax.step(t, logs_pid["C"],  color=colors['PID'], where='post', label='PID')
    ax.step(t, logs_str["C"],  color=colors['STR'], where='post', lw=2.0, label='STR (Adaptive)')
    ax.axhline(1.0, color="#333333", ls="--", label="Target (1.0%)", lw=1)
    ax.set(ylabel="Conc [%]", title="③ Salinity Concentration")
    ax.legend(loc='lower right')
    ax.grid(alpha=0.3)

    # ④ Electrical Conductivity
    ax = axes[1, 1]
    ax.plot(t, logs_open["sigma"], color=colors['OL'], alpha=0.6)
    ax.plot(t, logs_pid["sigma"],  color=colors['PID'], alpha=0.8)
    ax.plot(t, logs_str["sigma"],  color=colors['STR'])
    ax.set(ylabel="sigma [mS/cm]", title="④ Electrical Conductivity")
    ax.grid(alpha=0.3)

    # ⑤ Total Salt Added
    ax = axes[2, 0]
    ax.step(t, logs_open["salt_cum"], color=colors['OL'], where='post')
    ax.step(t, logs_pid["salt_cum"],  color=colors['PID'], where='post')
    ax.step(t, logs_str["salt_cum"],  color=colors['STR'], where='post')
    ax.set(xlabel="Time [s]", ylabel="Salt [g]", title="⑤ Total Salt Added")
    ax.grid(alpha=0.3)

    # ⑥ STR Parameter Estimation
    ax = axes[2, 1]
    if "alpha_hat" in logs_str and np.any(logs_str["alpha_hat"]):
        ax.plot(t, logs_str["alpha_hat"], color="#8B0000", lw=2, label=r"Estimated $\alpha$")
        ax.plot(t, logs_str["beta_hat"],  color="#FFD700", lw=2, label=r"Estimated $\beta$")
    ax.set(xlabel="Time [s]", title="⑥ STR Parameter Identification")
    ax.legend(loc='center right')
    ax.grid(alpha=0.3)

    plt.tight_layout()

# ============================================================
# 💡 まとめグラフ描画
# ============================================================
fig_err, ax_err = plt.subplots(figsize=(9, 6), num="Controller Robustness Summary")
soups_labels = ["Water", "Miso Soup", "Miso + Tofu"]
x_indices = np.arange(len(soups_labels))

ax_err.plot(x_indices, robustness_errors['OL'],  marker='o', markersize=10, linestyle='-', linewidth=2.5, color=colors['OL'], alpha=0.5, label='Open-Loop')
ax_err.plot(x_indices, robustness_errors['PID'], marker='s', markersize=10, linestyle='-', linewidth=2.5, color=colors['PID'], alpha=0.8, label='PID Control')
ax_err.plot(x_indices, robustness_errors['STR'], marker='*', markersize=14, linestyle='-', linewidth=2.5, color=colors['STR'], label='STR (Adaptive)')

ax_err.set_title("Controller Robustness against Soup Complexity", fontsize=15, fontweight='bold', pad=15)
ax_err.set_ylabel("Steady-State Relative Error [%]\n(% Deviation from Target Salinity)", fontsize=12)
ax_err.set_xticks(x_indices)
ax_err.set_xticklabels(soups_labels, fontsize=11)
ax_err.set_ylim(0, 50)  
ax_err.grid(axis='y', linestyle='--', alpha=0.4)
ax_err.legend(fontsize=12, loc='upper left')

ax_err.annotate('', xy=(0.95, -0.15), xytext=(0.05, -0.15), xycoords='axes fraction', textcoords='axes fraction',
                arrowprops=dict(arrowstyle="->", color='black', lw=2))
ax_err.text(0.5, -0.22, "Increasing Soup Complexity $\\rightarrow$", 
            ha='center', va='center', transform=ax_err.transAxes, fontsize=13, fontweight='bold')

plt.subplots_adjust(bottom=0.25)
plt.show()