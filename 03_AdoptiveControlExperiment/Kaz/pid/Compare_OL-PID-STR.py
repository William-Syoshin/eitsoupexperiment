import numpy as np
import matplotlib.pyplot as plt
from soup_plant import SoupPlant
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController # 前に作成したクラス

def run_simulation(
    control_mode="PID",
    salt_interval=30.0,
    salt_limit=1.0,
    temp_interval=10.0,
    kp_c=0.1, ki_c=0.005, kd_c=0.0,
    kp_t=0.05, ki_t=0.0005, kd_t=2.0
):
    DT = 1.0; SIM_TIME = 600.0; STEPS = int(SIM_TIME / DT)
    time = np.arange(STEPS) * DT
    T_REF = 50.0; C_REF = 1.0

    plant = SoupPlant(water_mass=500, potato_mass=100, T_w_init=20.0, T_p_init=20.0)
    rate_limit = salt_limit / salt_interval

    # 濃度コントローラの準備
    if control_mode == "OpenLoop":
        conc_controller = OpenLoopController(fixed_rate=0.0)
    else:
        # PID および STR はベースとしてこのPIDを使用
        conc_controller = PIDController(Kp=kp_c, Ki=ki_c, Kd=kd_c, output_min=0.0, output_max=rate_limit)

    # STRコントローラの準備 (STRモードの時のみ使用)
    str_unit = None
    if control_mode == "STR":
        # 初期値はフィッティング結果を使用
        str_unit = STRController(alpha_init=5, beta_init=1, a=0.02, T_base=25.0, lam=0.98)

    # 温度PID
    pid_temp = PIDController(Kp=kp_t, Ki=ki_t, Kd=kd_t, output_min=0.0)

    Q_in = 0.0; salt_added = 0.0
    logs = {
        "Tw": np.zeros(STEPS), "C": np.zeros(STEPS), "sigma": np.zeros(STEPS), 
        "Qin": np.zeros(STEPS), "salt_cum": np.zeros(STEPS),
        "alpha_hat": np.zeros(STEPS), "beta_hat": np.zeros(STEPS) # 推定値記録用
    }

    for i in range(STEPS):
        # 1. 温度制御
        if i % temp_interval == 0:
            e_T = T_REF - plant.T_w
            Q_in = max(0.0, pid_temp.compute(e_T, temp_interval))

        # 2. 濃度制御
        if i % salt_interval == 0:
            if control_mode == "OpenLoop":
                salt_added = 6.0 if i == 0 else 0.0
            elif control_mode == "PID":
                C_hat = plant.estimated_concentration
                e_C = C_REF - C_hat
                salt_added = conc_controller.compute(e_C, salt_interval) * salt_interval
            elif control_mode == "STR":
                # 【STRのキモ】まず現在のセンサー値からalphaとbetaを推定
                # plant.salt_mass / plant.water_mass を暫定濃度として入力
                a_hat, b_hat = str_unit.estimate(plant.conductivity, plant.concentration, plant.T_w)
                
                # 推定されたalphaに基づきゲインを自動調整
                kp_adj, ki_adj, kd_adj = str_unit.get_adjusted_gains(kp_c, ki_c, kd_c)
                conc_controller.Kp, conc_controller.Ki, conc_controller.Kd = kp_adj, ki_adj, kd_adj
                
                # 調整されたPIDで計算
                e_C = C_REF - plant.concentration
                salt_added = conc_controller.compute(e_C, salt_interval) * salt_interval
        else:
            salt_added = 0.0
       
        plant.step(Q_in=Q_in, salt_added=salt_added, dt=DT)
        
        # ログ記録
        logs["Tw"][i] = plant.T_w
        logs["C"][i] = plant.concentration
        logs["sigma"][i] = plant.conductivity
        logs["Qin"][i] = Q_in
        logs["salt_cum"][i] = plant.salt_mass
        if str_unit:
            logs["alpha_hat"][i] = str_unit.theta[0, 0]
            logs["beta_hat"][i] = str_unit.theta[1, 0]

    return time, logs

# ============================================================
# 実行と描画
# ============================================================
t, logs_pid  = run_simulation("PID")
_, logs_open = run_simulation("OpenLoop")
_, logs_str  = run_simulation("STR")

fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle("Performance Comparison: Open-Loop vs. PID vs. STR (Adaptive)", fontsize=16, fontweight="bold")

# 各種プロットの設定（共通）
styles = {'OL': {'color': 'gray', 'ls': '--', 'label': 'Open-Loop'},
          'PID': {'color': 'blue', 'ls': '-', 'label': 'Fixed PID'},
          'STR': {'color': 'red', 'ls': '-', 'label': 'STR (Adaptive)'}}

# ① Temperature
ax = axes[0, 0]
ax.plot(t, logs_open["Tw"], **styles['OL'], alpha=0.5)
ax.plot(t, logs_pid["Tw"], **styles['PID'], alpha=0.7)
ax.plot(t, logs_str["Tw"], **styles['STR'])
ax.axhline(50.0, color="black", ls=":", label="Target")
ax.set(ylabel="Temp [°C]", title="① Temperature Profile"); ax.legend(); ax.grid(alpha=0.3)

# ② Heating Power
ax = axes[0, 1]
ax.plot(t, logs_open["Qin"], **styles['OL'], alpha=0.5)
ax.plot(t, logs_pid["Qin"], **styles['PID'], alpha=0.7)
ax.plot(t, logs_str["Qin"], **styles['STR'])
ax.set(ylabel="Q_in [°C/s]", title="② Heating Power"); ax.grid(alpha=0.3)

# ③ Concentration
ax = axes[1, 0]
ax.plot(t, logs_open["C"], **styles['OL'], alpha=0.5)
ax.plot(t, logs_pid["C"], **styles['PID'], alpha=0.7)
ax.plot(t, logs_str["C"], **styles['STR'])
ax.axhline(1.0, color="black", ls=":", label="Target 1.0%")
ax.set(ylabel="Conc [%]", title="③ Salinity Concentration"); ax.grid(alpha=0.3)

# ④ Conductivity
ax = axes[1, 1]
ax.plot(t, logs_open["sigma"], **styles['OL'], alpha=0.5)
ax.plot(t, logs_pid["sigma"], **styles['PID'], alpha=0.7)
ax.plot(t, logs_str["sigma"], **styles['STR'])
ax.set(ylabel="sigma [mS/cm]", title="④ Electrical Conductivity"); ax.grid(alpha=0.3)

# ⑤ Total Salt
ax = axes[2, 0]
ax.plot(t, logs_open["salt_cum"], **styles['OL'], alpha=0.5)
ax.plot(t, logs_pid["salt_cum"], **styles['PID'], alpha=0.7)
ax.plot(t, logs_str["salt_cum"], **styles['STR'])
ax.set(xlabel="Time [s]", ylabel="Salt [g]", title="⑤ Total Salt Added"); ax.grid(alpha=0.3)

# ⑥ STR Parameter Estimation (見どころ！)
ax = axes[2, 1]
ax.plot(t, logs_str["alpha_hat"], color="darkred", label="Estimated alpha")
ax.plot(t, logs_str["beta_hat"], color="darkorange", label="Estimated beta")
ax.set(xlabel="Time [s]", title="⑥ STR Parameter Identification"); ax.legend(); ax.grid(alpha=0.3)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()