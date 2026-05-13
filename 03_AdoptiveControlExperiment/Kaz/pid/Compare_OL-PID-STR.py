import numpy as np
import matplotlib.pyplot as plt
from soup_plant import SoupPlant
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController # 前に作成したクラス

def run_simulation(
    control_mode="PID",
    salt_interval=30.0,
    salt_limit=1,
    temp_interval=1.0,
    kp_c=0.1, ki_c=0.005, kd_c=0.0,
    kp_t=0.05, ki_t=0.0005, kd_t=2.0
):
    # ── 1. 基本設定 ──
    DT = 1.0; SIM_TIME = 600.0; STEPS = int(SIM_TIME / DT)
    time = np.arange(STEPS) * DT
    T_REF = 50.0; C_REF = 1.0

    # 【ロボットの予習知識】水のみのフィッティング値
    ALPHA_NOMINAL = 8.836
    BETA_NOMINAL  = 0.499
    TEMP_COEFF    = 0.02
    T_BASE        = 25.0

    # プラント（現実）の生成
    plant = SoupPlant(water_mass=500, potato_mass=100, T_w_init=20.0, T_p_init=20.0)
    

    rate_limit = salt_limit / salt_interval

    # 濃度コントローラの準備
    if control_mode == "OpenLoop":
        conc_controller = OpenLoopController(fixed_rate=0.0)
    else:
        conc_controller = PIDController(Kp=kp_c, Ki=ki_c, Kd=kd_c, output_min=0.0, output_max=rate_limit)

    # STRの準備
    str_unit = None
    if control_mode == "STR":
        # STRも最初は ALPHA_NOMINAL（水の値）からスタート
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
        if i % salt_interval == 0:
            temp_comp = (1.0 + TEMP_COEFF * (plant.T_w - T_BASE))
            
            if control_mode == "OpenLoop":
                salt_added = 6.0 if i == 0 else 0.0
                
            elif control_mode == "PID":
                # 固定PIDは「水だけの時のALPHA_NOMINAL」を信じて疑わない
                # そのため、実際の濃度（plant.concentration）との間に認識のズレが生じる
                C_hat_fixed = (plant.conductivity / temp_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                e_C = C_REF - C_hat_fixed
                salt_added = conc_controller.compute(e_C, salt_interval) * salt_interval
                
            elif control_mode == "STR":
                # STRは現在のセンサー値から ALPHA と BETA をリアルタイム推定
                alpha_h, beta_h = str_unit.estimate(plant.conductivity, plant.concentration, plant.T_w)
                
                # 推定された alpha_h に基づきPIDゲインを自動調整
                kp_a, ki_a, kd_a = str_unit.get_adjusted_gains(kp_c, ki_c, kd_c)
                conc_controller.Kp, conc_controller.Ki, conc_controller.Kd = kp_a, ki_a, kd_a
                
                # 最新の推定パラメータ(alpha_h, beta_h)を使って、正しい濃度を把握する
                C_hat_adaptive = (plant.conductivity / temp_comp - beta_h) / alpha_h
                e_C = C_REF - C_hat_adaptive
                salt_added = conc_controller.compute(e_C, salt_interval) * salt_interval
        else:
            salt_added = 0.0
       
        plant.step(Q_in=Q_in, salt_added=salt_added, dt=DT)
        
        # 記録
        logs["Tw"][i], logs["C"][i], logs["sigma"][i] = plant.T_w, plant.concentration, plant.conductivity
        logs["Qin"][i], logs["salt_cum"][i] = Q_in, plant.salt_mass
        if str_unit:
            logs["alpha_hat"][i], logs["beta_hat"][i] = str_unit.theta[0, 0], str_unit.theta[1, 0]

    return time, logs

# ============================================================
# 実行と描画（ビビッドカラー・ラベル修正版）
# ============================================================
t, logs_pid  = run_simulation("PID")
_, logs_open = run_simulation("OpenLoop")
_, logs_str  = run_simulation("STR")

fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle("Performance Comparison: Open-Loop vs. PID vs. STR (Adaptive)", 
             fontsize=16, fontweight="bold")

# ビビッドなカラー設定
# STR: Vivid Orange, PID: Strong Blue, OL: Vivid Green
styles = {
    'OL':  {'color': '#00FF00', 'ls': '-', 'lw': 2, 'label': 'Open-Loop'}, # 鮮やかな緑
    'PID': {'color': '#0000FF', 'ls': '-', 'lw': 2, 'label': 'PID'},       # 強い青
    'STR': {'color': '#FF8C00', 'ls': '-', 'lw': 2.5, 'label': 'STR (Adaptive)'} # 強いオレンジ
}

# ① Temperature Profile
ax = axes[0, 0]
ax.plot(t, logs_open["Tw"], color=styles['OL']['color'], alpha=0.3)
ax.plot(t, logs_pid["Tw"],  color=styles['PID']['color'], alpha=0.4)
ax.plot(t, logs_str["Tw"],  color="#FF4500", lw=2) # 温度は少し赤に寄せたオレンジ
ax.axhline(50.0, color="black", ls=":", label="Target")
ax.set(ylabel="Temp [°C]", title="① Temperature Profile")
ax.grid(alpha=0.3)

# ② Heating Power (Qin: 紫系統を維持)
ax = axes[0, 1]
ax.plot(t, logs_open["Qin"], color="#E0B0FF", alpha=0.5) # ライトパープル
ax.plot(t, logs_pid["Qin"],  color="#9932CC", alpha=0.6) # ダークオーキッド
ax.plot(t, logs_str["Qin"],  color="#4B0082", lw=1.5)    # インディゴ
ax.set(ylabel="Q_in [°C/s]", title="② Manipulated Variable: Heating Power (Purple)")
ax.grid(alpha=0.3)

# ③ Salinity Concentration (ここで凡例を表示)
ax = axes[1, 0]
ax.plot(t, logs_open["C"], **styles['OL'])
ax.plot(t, logs_pid["C"],  **styles['PID'])
ax.plot(t, logs_str["C"],  **styles['STR'])
ax.axhline(1.0, color="#333333", ls="--", label="Target 1.0%", lw=1)
ax.set(ylabel="Conc [%]", title="③ Salinity Concentration")
ax.legend(loc='lower right', fontsize=10, frameon=True, shadow=True) # 凡例を強調
ax.grid(alpha=0.3)

# ④ Electrical Conductivity
ax = axes[1, 1]
ax.plot(t, logs_open["sigma"], **styles['OL'])
ax.plot(t, logs_pid["sigma"],  **styles['PID'])
ax.plot(t, logs_str["sigma"],  **styles['STR'])
ax.set(ylabel="sigma [mS/cm]", title="④ Electrical Conductivity")
ax.grid(alpha=0.3)

# ⑤ Total Salt Added
ax = axes[2, 0]
ax.plot(t, logs_open["salt_cum"], **styles['OL'])
ax.plot(t, logs_pid["salt_cum"],  **styles['PID'])
ax.plot(t, logs_str["salt_cum"],  **styles['STR'])
ax.set(xlabel="Time [s]", ylabel="Salt [g]", title="⑤ Total Salt Added")
ax.grid(alpha=0.3)

# ⑥ STR Parameter Estimation
ax = axes[2, 1]
ax.plot(t, logs_str["alpha_hat"], color="#8B0000", lw=2, label="Estimated alpha") # ダークレッド
ax.plot(t, logs_str["beta_hat"],  color="#FFD700", lw=2, label="Estimated beta")  # ゴールド
ax.set(xlabel="Time [s]", title="⑥ STR Parameter Identification")
ax.legend(loc='center right', fontsize=10)
ax.grid(alpha=0.3)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()

import matplotlib.pyplot as plt
import numpy as np

# --- 論文・フル幅（15cm）最適化設定 ---
plt.rcParams.update({
    "font.family": "serif",       # Times New Roman系
    "font.size": 10,              # 標準10pt
    "axes.labelsize": 10,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "lines.linewidth": 1.5,
})

# 1. 誤差データの計算
err_ol  = abs(logs_open["C"][-1] - 1.0)
err_pid = abs(logs_pid["C"][-1]  - 1.0)
err_str = abs(logs_str["C"][-1]  - 1.0)

errors = [err_ol, err_pid, err_str]
labels = ['Open-Loop', 'PID', 'Adaptive\n(STR)']
colors = ['#00FF00', '#0000FF', '#FF8C00'] # ビビッドカラー

# 2. 描画 (幅15cm = 5.9インチ, 高さ 6.5cm = 2.56インチ)
# 左右の幅比率を 1.6 : 1 にして、メインのグラフを大きく見せます
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15/2.54, 6.5/2.54), 
                               gridspec_kw={'width_ratios': [1.6, 1]})

# --- (a) Time-series Response ---
ax1.plot(t, logs_open["C"], color=colors[0], label='Open-Loop')
ax1.plot(t, logs_pid["C"],  color=colors[1], label='PID')
ax1.plot(t, logs_str["C"],  color=colors[2], lw=2.0, label='Adaptive (STR)')
ax1.axhline(1.0, color="#333333", ls="--", lw=1.0, label="Target")

ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Salinity Conc. [%]")
ax1.set_title("(a) Time-series Response", fontsize=10, fontweight='bold')
ax1.legend(loc='lower right', frameon=True, fontsize=8)
ax1.grid(alpha=0.3)

# --- (b) Steady-state Error ---
bars = ax2.bar(labels, errors, color=colors, alpha=0.8, edgecolor='black', linewidth=0.8)
ax2.set_ylabel("Final Error [%]")
ax2.set_title("(b) Steady-state Error", fontsize=10, fontweight='bold')
ax2.set_ylim(0, 0.3) # 誤差を見やすく固定
ax2.grid(axis='y', alpha=0.3)

# バーの上に値を表示
for bar in bars:
    height = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2., height + 0.01,
             f'{height:.2f}', ha='center', va='bottom', fontsize=8)

plt.tight_layout(pad=1.2)

# 保存
plt.savefig("fig6_7_combined_15cm.svg", format="svg")
plt.savefig("fig6_7_combined_15cm.png", format="png", dpi=300)
plt.show()