import numpy as np
import matplotlib.pyplot as plt

# 既存のクラスをインポート（フォルダ構成に合わせて適宜調整してください）
from soup_plant import SoupPlant
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController

def run_single_simulation(env_params, control_mode):
    """
    指定された環境パラメータと制御モードで1回シミュレーションを回し、
    最終的な塩分濃度の誤差（|C - 1.0| %）を返す関数
    """
    # ── 基本設定 ──
    DT = 1.0
    SIM_TIME = 600.0  # 10分間
    STEPS = int(SIM_TIME / DT)
    C_REF = 1.0       # 目標濃度 1.0%
    
    # コントローラが信じている「純水」の公称パラメータ
    ALPHA_NOMINAL = 8.836
    BETA_NOMINAL  = 0.499
    T_BASE = 25.0
    A_TEMP = 0.02
    
    # ── プラント（現実の環境）のセットアップ ──
    plant = SoupPlant(
        water_mass=env_params["m_w"], 
        potato_mass=env_params["m_p"], 
        T_w_init=20.0, 
        T_p_init=20.0
    )
    # 環境ごとの電気伝導率特性（ズレ）を上書き設定
    plant.alpha = env_params["alpha"]
    plant.beta = env_params["beta"]

    # ── コントローラのセットアップ ──
    if control_mode == "OL":
        # オープンループ: 最初の1ステップで6.0gの塩を投入するだけの制御を想定
        # (総質量600gの純水ならピッタリ1.0%になる計算)
        salt_to_add = 6.0
    elif control_mode == "PID":
        pid_c = PIDController(Kp=0.1, Ki=0.005, Kd=0.0, output_min=0.0, output_max=1.0)
    elif control_mode == "STR":
        str_c = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, a=A_TEMP, T_base=T_BASE)
        pid_c = PIDController(Kp=0.1, Ki=0.005, Kd=0.0, output_min=0.0, output_max=1.0)

    # ヒーター用のPID（濃度制御の比較がメインなので共通）
    pid_t = PIDController(Kp=0.05, Ki=0.0005, Kd=2.0, output_min=0.0, output_max=2.0)

    # ── シミュレーションループ ──
    for step in range(STEPS):
        # センサー計測
        T_meas = plant.T_w
        sigma_meas = plant.conductivity
        
        # 1. ヒーター制御 (温度を50℃へ)
        error_t = 50.0 - T_meas
        Q_in = pid_t.compute(error_t, DT)
        
        # 2. 塩分投入制御 (30秒に1回)
        salt_added = 0.0
        if step % 30 == 0:
            if control_mode == "OL":
                if step == 0:
                    salt_added = salt_to_add # 最初だけ6g入れる
            
            elif control_mode == "PID":
                # 固定パラメータを信じて現在の濃度を逆算（ここで認識ズレが起きる）
                C_est = (sigma_meas / (1.0 + A_TEMP * (T_meas - T_BASE)) - BETA_NOMINAL) / ALPHA_NOMINAL
                error_c = C_REF - C_est
                salt_added = pid_c.compute(error_c, 30.0)
                
            elif control_mode == "STR":
                # 適応制御: パラメータをリアルタイム推定
                # (ここでは投入済み総塩分量からC_totalを計算していると仮定)
                C_total = (plant.salt_mass / plant.liquid_mass) * 100.0 if plant.liquid_mass > 0 else 0.0
                str_c.estimate(sigma_meas, C_total, T_meas)
                
                # 推定したαとβを使って正確な現在濃度を逆算
                alpha_hat = str_c.theta[0, 0]
                beta_hat = str_c.theta[1, 0]
                C_est = (sigma_meas / (1.0 + A_TEMP * (T_meas - T_BASE)) - beta_hat) / alpha_hat
                
                error_c = C_REF - C_est
                salt_added = pid_c.compute(error_c, 30.0)

        # プラントの更新
        plant.step(Q_in=Q_in, salt_added=salt_added, dt=DT)

    # 最終的な実際の塩分濃度との誤差（絶対値）を返す
    final_error = abs(plant.concentration - C_REF)
    return final_error

# ==========================================
# メイン処理：各環境・各制御手法でシミュレーションを実行
# ==========================================

# 3つの環境（スープの複雑さ）を定義
environments = [
    {
        "name": "Pure Water\n(純水)", 
        "m_w": 600, "m_p": 0,       # 水のみ600g
        "alpha": 8.836, "beta": 0.499 # コントローラの公称値と完全一致
    },
    {
        "name": "Water + Potato\n(水＋芋)", 
        "m_w": 500, "m_p": 100,     # 水500g + 芋100g（総質量は同じだが液相が減る）
        "alpha": 11.925, "beta": 0.6  # 現実の特性が少しズレる
    },
    {
        "name": "Miso Soup\n(味噌スープ)", 
        "m_w": 500, "m_p": 100, 
        "alpha": 18.0, "beta": 1.5    # 味噌の成分で電気伝導率が激しくズレる（ダミー値）
    }
]

modes = ["OL", "PID", "STR"]
results = {mode: [] for mode in modes}

print("シミュレーションを実行中...")
for env in environments:
    print(f"[{env['name'].replace(chr(10), ' ')}]")
    for mode in modes:
        error = run_single_simulation(env, mode)
        results[mode].append(error)
        print(f"  {mode} Error: {error:.4f} %")

# ==========================================
# グラフの描画
# ==========================================
plt.style.use('default')
fig, ax = plt.subplots(figsize=(8, 5))

categories = [env["name"] for env in environments]
x_pos = np.arange(len(categories))

# プロット
ax.plot(x_pos, results["OL"], marker='o', markersize=8, linestyle='-', linewidth=2, 
        color='#E74C3C', label='Open-Loop')
ax.plot(x_pos, results["PID"], marker='s', markersize=8, linestyle='-', linewidth=2, 
        color='#2980B9', label='PID')
ax.plot(x_pos, results["STR"], marker='^', markersize=8, linestyle='-', linewidth=2.5, 
        color='#27AE60', label='Adaptive (STR)')

# 装飾
ax.set_xticks(x_pos)
ax.set_xticklabels(categories, fontsize=11, fontweight='bold')
ax.set_ylabel("Final Steady-State Error [%]", fontsize=12)
ax.set_title("Controller Robustness vs. Soup Complexity", fontsize=14, fontweight='bold')
ax.grid(alpha=0.3, linestyle='--')
ax.legend(fontsize=11, loc='upper left')

# X軸の下に矢印を追加
ax.annotate('', xy=(0.85, -0.15), xycoords='axes fraction', 
            xytext=(0.15, -0.15), textcoords='axes fraction',
            arrowprops=dict(arrowstyle="->", color='black', lw=1.5))
ax.text(0.5, -0.18, 'Soup Complexity (Large) ->', 
        ha='center', va='top', transform=ax.transAxes, fontsize=11, fontweight='bold')

plt.tight_layout()
plt.subplots_adjust(bottom=0.2) 

plt.savefig("Fig_RealSim_Complexity_vs_Error.png", dpi=300)
print("グラフを 'Fig_RealSim_Complexity_vs_Error.png' として保存しました！")
plt.show()