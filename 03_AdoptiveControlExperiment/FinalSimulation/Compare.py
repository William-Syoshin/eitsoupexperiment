import numpy as np
import matplotlib.pyplot as plt
from Soup_Plant import SoupPlant
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController

def run_simulation(
    soup_type="potato",       # スープの種類
    control_mode="PID",
    salt_interval=30.0,
    salt_limit=1.0,
    temp_interval=1.0,
    kp_c=0.1, ki_c=0.005, kd_c=0.0,
    kp_t=0.05, ki_t=0.0005, kd_t=2.0
):
    # ── 基本設定 ──
    DT = 1.0; SIM_TIME = 600.0; STEPS = int(SIM_TIME / DT)
    time = np.arange(STEPS) * DT
    T_REF = 50.0; C_REF = 1.0

    # 【ロボットの予習知識】水のみのフィッティング値
    ALPHA_NOMINAL = 11.9320
    BETA_NOMINAL  = 0.3410
    TEMP_COEFF    = 0.02
    T_BASE        = 25.0

    # プラント（現実の鍋）の生成
    plant = SoupPlant(soup_type=soup_type, T_w_init=20.0, T_s_init=20.0, T_room=20.0)
    rate_limit = salt_limit / salt_interval

    # 💡【修正ポイント】実行ごとに毎回完全に独立した「新品」のコントローラーを生成する
    if control_mode == "OpenLoop":
        conc_controller = OpenLoopController(fixed_rate=0.0)
    else:
        # PIDでもSTRでも、最初は「初期ゲイン（kp_c, ki_c, kd_c）」を持った新品のPIDを用意する
        conc_controller = PIDController(Kp=kp_c, Ki=ki_c, Kd=kd_c, output_min=0.0, output_max=rate_limit)

    # STRの準備
    str_unit = None
    if control_mode == "STR":
        str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, a=TEMP_COEFF, T_base=T_BASE, lam=0.98)

    # 温度PID
    pid_temp = PIDController(Kp=kp_t, Ki=ki_t, Kd=kd_t, output_min=0.0)

    # ログ初期化
    logs = {key: np.zeros(STEPS) for key in ["Tw", "C", "sigma", "Qin", "salt_cum", "alpha_hat", "beta_hat"]}

    for i in range(STEPS):
        # --- 1. 温度制御 ---
        if i % temp_interval == 0:
            e_T = T_REF - plant.T_w
            Q_in = max(0.0, pid_temp.compute(e_T, temp_interval))

        # --- 2. 濃度制御 ---
        # 💡 「and i > 0」を追加して、0秒目（i=0）は何もせずスルーさせる
        if i % salt_interval == 0 and i > 0:
            temp_comp = (1.0 + TEMP_COEFF * (plant.T_w - T_BASE))
            
            if control_mode == "OpenLoop":
                # 💡 OpenLoopの投入タイミングも「最初(0秒)」から「1回目のインターバル(30秒)」に変更
                salt_added = 6.0 if i == salt_interval else 0.0
                
            elif control_mode == "PID":
                C_hat_fixed = (plant.conductivity / temp_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                e_C = C_REF - C_hat_fixed
                salt_added = conc_controller.compute(e_C, salt_interval) * salt_interval
                
            elif control_mode == "STR":
                alpha_h, beta_h = str_unit.estimate(plant.conductivity, plant.concentration, plant.T_w)
                kp_a, ki_a, kd_a = str_unit.get_adjusted_gains(kp_c, ki_c, kd_c)
                
                conc_controller.Kp = kp_a
                conc_controller.Ki = ki_a
                conc_controller.Kd = kd_a
                
                C_hat_adaptive = (plant.conductivity / temp_comp - beta_h) / alpha_h
                e_C = C_REF - C_hat_adaptive
                salt_added = conc_controller.compute(e_C, salt_interval) * salt_interval
        else:
            salt_added = 0.0
       
        plant.step(Q_in=Q_in, salt_added=salt_added, dt=DT)
        
        # 記録
        logs["Tw"][i] = plant.T_w
        logs["C"][i] = plant.concentration
        logs["sigma"][i] = plant.conductivity
        logs["Qin"][i] = Q_in
        logs["salt_cum"][i] = plant.salt_mass
        if str_unit:
            logs["alpha_hat"][i] = str_unit.theta[0, 0] if hasattr(str_unit, 'theta') else alpha_h
            logs["beta_hat"][i] = str_unit.theta[1, 0] if hasattr(str_unit, 'theta') else beta_h

    return time, logs


# ============================================================
# ⚙️ 描画処理：3つのスープ×6パネル ＋ 🛠️【修正版】定常エラー評価グラフ
# ============================================================
colors = {'OL': '#00FF00', 'PID': '#0000FF', 'STR': '#FF8C00'}
target_soups = ["water", "potato", "miso_tofu"]

# 💡 実行するたびに確実にリストをリセット
robustness_errors = {'OL': [], 'PID': [], 'STR': []}

C_REF = 1.0
EVAL_START_TIME = 500.0  # 完全に安定した最後の100秒のみを評価

# スープごとにループを回して、シミュレーション・エラー計算・6パネル描画を全てここで済ませる
for soup in target_soups:
    print(f"Simulating {soup}...")
    
    # --- シミュレーション実行 ---
    t, logs_open = run_simulation(soup_type=soup, control_mode="OpenLoop", kp_c=0.1, ki_c=0.005, kd_c=0.0)
    _, logs_pid  = run_simulation(soup_type=soup, control_mode="PID",      kp_c=0.1, ki_c=0.005, kd_c=0.0)
    _, logs_str  = run_simulation(soup_type=soup, control_mode="STR",      kp_c=0.1, ki_c=0.005, kd_c=0.0)
    
    # 💡 相対誤差（Relative Error [%]）の平均を算出してリストに追加
    # 計算式： (|実際の濃度 - 目標濃度| / 目標濃度) * 100
    idx = (t >= EVAL_START_TIME)
    robustness_errors['OL'].append(np.mean(np.abs(logs_open["C"][idx] - C_REF)) / C_REF * 100.0)
    robustness_errors['PID'].append(np.mean(np.abs(logs_pid["C"][idx] - C_REF)) / C_REF * 100.0)
    robustness_errors['STR'].append(np.mean(np.abs(logs_str["C"][idx] - C_REF)) / C_REF * 100.0)
    
    # --- 6パネルグラフの描画 ---
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
    ax.plot(t, logs_str["alpha_hat"], color="#8B0000", lw=2, label=r"Estimated $\alpha$")
    ax.plot(t, logs_str["beta_hat"],  color="#FFD700", lw=2, label=r"Estimated $\beta$")
    ax.set(xlabel="Time [s]", title="⑥ STR Parameter Identification")
    ax.legend(loc='center right')
    ax.grid(alpha=0.3)

    plt.tight_layout()

# ============================================================
# 💡 まとめグラフ描画部分（最後に1回だけ実行）
# ============================================================
fig_err, ax_err = plt.subplots(figsize=(9, 6), num="Controller Robustness Summary")
soups_labels = ["Water", "Potato Soup", "Miso Tofu Soup"]
x_indices = np.arange(len(soups_labels))

# 各コントローラーのエラーの軌跡をプロット
ax_err.plot(x_indices, robustness_errors['OL'],  marker='o', markersize=10, linestyle='-', linewidth=2.5, color=colors['OL'], alpha=0.5, label='Open-Loop')
ax_err.plot(x_indices, robustness_errors['PID'], marker='s', markersize=10, linestyle='-', linewidth=2.5, color=colors['PID'], alpha=0.8, label='PID Control')
ax_err.plot(x_indices, robustness_errors['STR'], marker='*', markersize=14, linestyle='-', linewidth=2.5, color=colors['STR'], label='STR (Adaptive)')

# グラフの装飾
ax_err.set_title("Controller Robustness against Soup Complexity", fontsize=15, fontweight='bold', pad=15)
ax_err.set_ylabel("Steady-State Relative Error [%]\n(% Deviation from Target Salinity)", fontsize=12)
ax_err.set_xticks(x_indices)
ax_err.set_xticklabels(soups_labels, fontsize=11)
ax_err.set_ylim(0, 50)  # 誤差100%が収まるように120%へ拡張
ax_err.grid(axis='y', linestyle='--', alpha=0.4)
ax_err.legend(fontsize=12, loc='upper left')

# X軸の下に矢印と文字を追加
ax_err.annotate('', xy=(0.95, -0.15), xytext=(0.05, -0.15), xycoords='axes fraction', textcoords='axes fraction',
                arrowprops=dict(arrowstyle="->", color='black', lw=2))
ax_err.text(0.5, -0.22, "Increasing Soup Complexity $\\rightarrow$", 
            ha='center', va='center', transform=ax_err.transAxes, fontsize=13, fontweight='bold')

plt.subplots_adjust(bottom=0.25)
plt.show()