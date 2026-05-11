import numpy as np
import matplotlib.pyplot as plt
from soup_plant import SoupPlant

# --- インポートのパスに注意 (あなたの環境に合わせています) ---
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController

def run_simulation(
    control_mode="PID",
    control_interval=30.0,
    salt_limit=1.0,        # 一回に入れられる最大量 [g]
    kp=0.1, ki=0.005, kd=0.0
):
    # ── 基本設定 ──
    DT       = 1.0    # 物理計算の刻み
    SIM_TIME = 600.0  # 全体10分間
    STEPS    = int(SIM_TIME / DT)
    time     = np.arange(STEPS) * DT
    T_REF    = 50.0   # 目標温度
    C_REF    = 1.0    # 目標濃度

    # プラント生成 (物理定数は soup_plant.py に準拠)
    # alpha=8.836, beta=0.499, k_loss=0.0017
    plant = SoupPlant(water_mass=500, potato_mass=100, T_w_init=20.0, T_p_init=20.0)
    
    # 投入レートの上限を計算
    rate_limit = salt_limit / control_interval

    # 濃度PID（output_maxを設定）
    if control_mode == "PID":
        conc_controller = PIDController(
            Kp=kp, Ki=ki, Kd=kd, 
            output_min=0.0, 
            output_max=rate_limit
        )
    else:
        conc_controller = OpenLoopController(fixed_rate=0.0)

    # 温度PID
    pid_temp = PIDController(Kp=0.05, Ki=0.0005, Kd=2.0, output_min=0.0)

    # 記録用
    Q_in = 0.0; salt_added = 0.0
    logs = {
        "Tw": np.zeros(STEPS), "C": np.zeros(STEPS), "sigma": np.zeros(STEPS), 
        "Qin": np.zeros(STEPS), "salt_cum": np.zeros(STEPS)
    }

    for i in range(STEPS):
        if i % control_interval == 0:
            # 1. 温度更新
            e_T = T_REF - plant.T_w
            Q_in = max(0.0, pid_temp.compute(e_T, control_interval))

            # 2. 濃度更新
            if control_mode == "OpenLoop":
                salt_added = 6.0 if i == 0 else 0.0 # 初期投入6g
            elif control_mode == "PID":
                C_hat = plant.estimated_concentration
                e_C = C_REF - C_hat
                # コントローラ内部で上限が適用される
                salt_added = conc_controller.compute(e_C, control_interval) * control_interval
        else:
            salt_added = 0.0
       
        plant.step(Q_in=Q_in, salt_added=salt_added, dt=DT)
        
        logs["Tw"][i] = plant.T_w
        logs["C"][i] = plant.concentration
        logs["sigma"][i] = plant.conductivity
        logs["Qin"][i] = Q_in
        logs["salt_cum"][i] = plant.salt_mass

    return time, logs

# ============================================================
#シミュレーションの実行
# ============================================================
# 1. PID制御の結果を取得
time, logs_pid = run_simulation(control_mode="PID", control_interval=30.0, salt_limit=1.0)

# 2. オープンループの結果を取得
_, logs_open = run_simulation(control_mode="OpenLoop")

# ============================================================
# グラフ描画 (3行2列)
# ============================================================
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle("Detailed Comparison: PID Control vs. Open-Loop\n(Interval: 30s, Salt Limit: 1.0g)", 
             fontsize=15, fontweight="bold")

# ① Temperature Profile
ax = axes[0, 0]
ax.plot(time, logs_pid["Tw"], color="#C0392B", lw=2, label="PID (Water)")
ax.plot(time, logs_open["Tw"], color="#C0392B", ls="--", alpha=0.5, label="Open-Loop")
ax.axhline(50.0, color="gray", ls=":", label="Target 50°C")
ax.set(ylabel="Temp [°C]", title="① Temperature Profile")
ax.legend(fontsize=9); ax.grid(alpha=0.3)

# ② Heating Power Qin
ax = axes[0, 1]
ax.plot(time, logs_pid["Qin"], color="#8E44AD", lw=1.5, label="PID (Power)")
ax.plot(time, logs_open["Qin"], color="#8E44AD", ls="--", alpha=0.4, label="Open-Loop")
ax.set(ylabel="Q_in [°C/s]", title="② Manipulated Variable: Heating Power")
ax.legend(fontsize=9); ax.grid(alpha=0.3)

# ③ Salinity Concentration
ax = axes[1, 0]
ax.plot(time, logs_pid["C"], color="#27AE60", lw=2, label="PID (Conc)")
ax.plot(time, logs_open["C"], color="#2980B9", lw=2, ls="--", label="Open-Loop (6g Initial)")
ax.axhline(1.0, color="black", ls=":", label="Target 1.0%")
ax.set(ylabel="Conc [%]", title="③ Salinity Concentration")
ax.legend(fontsize=9); ax.grid(alpha=0.3)

# ④ Electrical Conductivity
ax = axes[1, 1]
ax.plot(time, logs_pid["sigma"], color="#2980B9", lw=2, label="PID (sigma)")
ax.plot(time, logs_open["sigma"], color="#2980B9", ls="--", alpha=0.5, label="Open-Loop")
ax.set(ylabel="sigma [mS/cm]", title="④ Electrical Conductivity")
ax.legend(fontsize=9); ax.grid(alpha=0.3)

# ⑤ Total Salt Added
ax = axes[2, 0]
ax.plot(time, logs_pid["salt_cum"], color="#D35400", lw=2, label="PID (Total Salt)")
ax.plot(time, logs_open["salt_cum"], color="#D35400", ls="--", label="Open-Loop")
ax.set(xlabel="Time [s]", ylabel="Cumulative Salt [g]", title="⑤ Total Salt Added")
ax.legend(fontsize=9); ax.grid(alpha=0.3)

# ⑥ 空き
axes[2, 1].set_axis_off()

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()