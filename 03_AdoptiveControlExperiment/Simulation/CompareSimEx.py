# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from Controls.PIDcontrol import PIDController
from Controls.AdaptiveControl import STRController

# ==========================================
# 1. 共通定数・シミュレーション関数（軌跡取得用）
# ==========================================
MAX_STEPS = 24
SALT_INTERVAL = 30.0
C_TARGET = 1.0
SALT_MAX_PER_STEP = 0.5
M_TOTAL_ASSUMED = 600.0  

ALPHA_NOMINAL = 8.1013
BETA_NOMINAL  = 0.1550
TEMP_COEFF    = 0.02
T_BASE        = 24.6  # 基準温度（補正用）

ALPHA_MIN, ALPHA_MAX = ALPHA_NOMINAL / 3.0, ALPHA_NOMINAL * 3.0
BETA_MIN, BETA_MAX = 0.0, 5.0
D_ALPHA_MAX, D_BETA_MAX = 0.5, 0.2
DEADBAND = 0.02
KP, KI, KD = 0.1, 0.00, 0.0
T_CONST = 50.0

class FastSoupPlant:
    def __init__(self, m_water=500.0, initial_salt_pct=0.35, true_alpha=8.5359, true_beta=1.1373):
        self.m_liquid_base = m_water 
        self.salt_mass = self.m_liquid_base * (initial_salt_pct / 100.0)
        self.added_salt = 0.0
        self.true_alpha = true_alpha
        self.true_beta = true_beta
        
    def add_salt(self, target_salt_g):
        actual_salt_g = target_salt_g * np.random.normal(loc=1.0, scale=0.05)
        self.added_salt += actual_salt_g
        self.salt_mass += actual_salt_g
        
    @property
    def true_concentration(self):
        return (self.salt_mass / (self.m_liquid_base + self.added_salt)) * 100.0
    
    def get_ec(self, current_temp):
        # 🌟 基準温度(24.6℃)におけるベース伝導率
        sigma_base = self.true_alpha * self.true_concentration + self.true_beta
        # 🌟 実際の温度(current_temp)における伝導率に変換（冷めると下がる）
        true_ec = sigma_base * (1.0 + TEMP_COEFF * (current_temp - T_BASE))
        noisy_ec = true_ec * np.random.normal(loc=1.0, scale=0.01)
        return noisy_ec

def run_simulation_trace(mode="STR"):
    # Miso+Tofu の環境
    plant = FastSoupPlant(m_water=500.0, initial_salt_pct=0.35)
    str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, lam=1.0)
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD, output_min=0.0, output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)
    
    total_salt = 0.0
    alpha_h, beta_h = ALPHA_NOMINAL, BETA_NOMINAL
    prev_alpha, prev_beta = ALPHA_NOMINAL, BETA_NOMINAL
    C_init_virtual = 0.0
    
    # 🌟 目標温度（保温制御） 50℃
    target_temp = 50.0
    current_temp = 50.0 
    
    history_cond = []
    history_temp = []
    history_salt = []
    history_conc = []
    
    for step in range(MAX_STEPS):
        # 🌟 ヒーターによる保温制御を模倣（50℃付近で揺らぐノイズ）
        current_temp = target_temp + np.random.normal(0, 0.5)
        
        raw_ec = plant.get_ec(current_temp)
        # コントローラは基準温度（24.6℃）換算に補正してから計算する
        sigma_comp = raw_ec / (1.0 + TEMP_COEFF * (current_temp - T_BASE))
        
        # 記録
        history_temp.append(current_temp)
        history_cond.append(raw_ec)
        history_conc.append(plant.true_concentration)
        
        salt_g = 0.0
        
        if mode == "PID":
            C_hat = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
            salt_g = pid.compute(C_TARGET - C_hat, SALT_INTERVAL) * SALT_INTERVAL
            
        elif mode == "STR":
            if total_salt < 0.01:
                C_init_virtual = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                prev_beta = BETA_NOMINAL
            
            X_virtual_total = C_init_virtual + (total_salt / M_TOTAL_ASSUMED) * 100.0
            raw_alpha_h, raw_beta_h = str_unit.estimate(sigma_comp, X_virtual_total)
            
            d_alpha = max(-D_ALPHA_MAX, min(raw_alpha_h - prev_alpha if abs(raw_alpha_h - prev_alpha) >= DEADBAND else 0.0, D_ALPHA_MAX))
            d_beta  = max(-D_BETA_MAX,  min(raw_beta_h - prev_beta  if abs(raw_beta_h - prev_beta)  >= DEADBAND else 0.0, D_BETA_MAX))
            
            alpha_h = max(ALPHA_MIN, min(prev_alpha + d_alpha, ALPHA_MAX))
            beta_h  = max(BETA_MIN,  min(prev_beta  + d_beta,  BETA_MAX))
            prev_alpha, prev_beta = alpha_h, beta_h
            
            adapt_ratio = min(max(ALPHA_NOMINAL / alpha_h, 0.3), 3.0)
            pid.Kp, pid.Ki, pid.Kd = KP * adapt_ratio, KI * adapt_ratio, KD * adapt_ratio
            
            C_true_abs = C_init_virtual if total_salt < 0.01 else (sigma_comp - beta_h) / alpha_h
            salt_g = pid.compute(C_TARGET - C_true_abs, SALT_INTERVAL) * SALT_INTERVAL
            
        salt_g = max(0.0, min(salt_g, SALT_MAX_PER_STEP))
        history_salt.append(salt_g)
        
        if salt_g >= 0.01:
            plant.add_salt(salt_g)
            total_salt += salt_g

    return history_cond, history_temp, history_salt, history_conc

# シミュレーションの実行
sim_cond_pid, sim_temp_pid, sim_salt_pid, sim_conc_pid = run_simulation_trace(mode="PID")
sim_cond_str, sim_temp_str, sim_salt_str, sim_conc_str = run_simulation_trace(mode="STR")


# ==========================================
# 2. 実機データ（モック）の準備
# ==========================================
# ユーザーから提供された STR の実機データ（10ステップ分）
miso_str_raw = [0.270, 0.338, 0.418, 0.507, 0.594, 0.685, 0.771, 0.833, 0.834, 0.835]
exp_conc_str = miso_str_raw + [miso_str_raw[-1]] * (24 - len(miso_str_raw))

# TODO: 以下の実機データはダミーです。本物の実験データ配列に差し替えてください。
exp_cond_pid = sim_cond_pid  # 本物のPIDのECデータ
exp_cond_str = sim_cond_str  # 本物のSTRのECデータ
exp_temp_pid = sim_temp_pid  # 本物のPIDの温度データ
exp_temp_str = sim_temp_str  # 本物のSTRの温度データ
exp_salt_pid = sim_salt_pid  # 本物のPIDの塩投入量データ
exp_salt_str = sim_salt_str  # 本物のSTRの塩投入量データ
# PIDはオーバーシュートして1.3%くらいで止まるダミーデータ
exp_conc_pid = [0.270 + i*0.06 for i in range(15)] + [1.17] * 9  

# 行列構造への整理
# 列0: Simulation, 列1: Experiment
steps = list(range(1, 25))
grid_cond = [(sim_cond_pid, sim_cond_str), (exp_cond_pid, exp_cond_str)]
grid_temp = [(sim_temp_pid, sim_temp_str), (exp_temp_pid, exp_temp_str)]
grid_salt = [(sim_salt_pid, sim_salt_str), (exp_salt_pid, exp_salt_str)]
grid_conc = [(sim_conc_pid, sim_conc_str), (exp_conc_pid, exp_conc_str)]


# ==========================================
# 3. 描画設定（3行2列）
# ==========================================
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['axes.labelsize'] = 11

fig_width_in = 15 / 2.54   # 論文幅 15cm
fig_height_in = 14 / 2.54  # 縦幅（少しゆとりを持たせる）

fig, axes = plt.subplots(3, 2, figsize=(fig_width_in, fig_height_in), 
                         gridspec_kw={'height_ratios': [1, 1, 1.4], 'hspace': 0.35, 'wspace': 0.35})

colors = {'PID': '#1f77b4', 'STR': '#ce0000', 'Temp': '#ff7f0e'}
styles = {
    'PID': dict(color=colors['PID'], linestyle='--', linewidth=1.8),
    'STR': dict(color=colors['STR'], linestyle='-',  linewidth=2.2),
}

col_titles = ["Simulation (Miso+Tofu)", "Experiment (Miso+Tofu)"]

for col in range(2):
    # ---------------------------------------------------
    # 1段目: 伝導率 と 温度 (2軸グラフ)
    # ---------------------------------------------------
    ax0 = axes[0, col]
    ax0.set_title(col_titles[col], fontweight='bold', pad=10)
    
    # 伝導率（左軸）
    ax0.plot(steps, grid_cond[col][0], label='EC (PID)', **styles['PID'])
    ax0.plot(steps, grid_cond[col][1], label='EC (STR)', **styles['STR'])
    ax0.set_ylim(0, 15)
    ax0.set_xticks([1, 6, 12, 18, 24])
    ax0.set_xticklabels([]) # x軸ラベルは隠す
    ax0.grid(alpha=0.3)
    
    # 温度（右軸）
    ax0_twin = ax0.twinx()
    ax0_twin.plot(steps, grid_temp[col][1], color=colors['Temp'], linestyle=':', linewidth=2.0, alpha=0.8, label='Temperature')
    ax0_twin.set_ylim(40, 60)
    
    if col == 0:
        ax0.set_ylabel(r"EC $\sigma$ [mS/cm]")
    if col == 1:
        ax0_twin.set_ylabel(r"Temperature [$^\circ$C]")
    else:
        ax0_twin.set_yticklabels([]) # 左側のグラフの右軸数値は隠す
        
    # ---------------------------------------------------
    # 2段目: 塩の追加量 (面グラフ/塗りつぶし)
    # ---------------------------------------------------
    ax1 = axes[1, col]
    # PIDはステップラインで表示
    ax1.step(steps, grid_salt[col][0], where='mid', label='PID', **styles['PID'])
    # STRは目立つように面グラフ（fill_between）で表示
    ax1.fill_between(steps, grid_salt[col][1], step="mid", color=colors['STR'], alpha=0.3, label='STR Added')
    ax1.step(steps, grid_salt[col][1], where='mid', color=colors['STR'], linewidth=1.5)
    
    ax1.set_ylim(0, 0.6)
    ax1.set_yticks([0, 0.25, 0.5])
    ax1.set_xticks([1, 6, 12, 18, 24])
    ax1.set_xticklabels([])
    ax1.grid(alpha=0.3)
    
    if col == 0:
        ax1.set_ylabel("Added Salt [g]")

    # ---------------------------------------------------
    # 3段目: 最終濃度
    # ---------------------------------------------------
    ax2 = axes[2, col]
    ax2.axhline(1.0, color='black', linestyle=':', alpha=0.6, label='Target (1.0%)')
    
    ax2.plot(steps, grid_conc[col][0], label='Fixed PID', **styles['PID'])
    ax2.plot(steps, grid_conc[col][1], label='Adaptive Tasting (STR)', **styles['STR'])
    
    ax2.set_ylim(0, 1.4)
    ax2.set_yticks([0, 0.5, 1.0])
    ax2.set_xticks([1, 6, 12, 18, 24])
    ax2.grid(alpha=0.3)
    ax2.set_xlabel("Step")
    
    if col == 0:
        ax2.set_ylabel("Concentration [%]")

# ==========================================
# 4. 凡例とキャプションのレイアウト調整
# ==========================================
plt.subplots_adjust(left=0.12, right=0.90, top=0.90, bottom=0.18)

# 凡例の収集
handles_cond, labels_cond = axes[0,0].get_legend_handles_labels()
handles_temp, labels_temp = axes[0,0].twinx().get_legend_handles_labels()
handles_conc, labels_conc = axes[2,0].get_legend_handles_labels()

# 図全体の一番下に共通の凡例を配置
fig.legend(handles_cond + handles_temp + handles_conc, 
           labels_cond + labels_temp + labels_conc, 
           loc='lower center', bbox_to_anchor=(0.5, 0.02), ncol=3, fontsize=10, frameon=False)

filename_png = 'Sim_Exp_Comparison.png'
filename_pdf = 'Sim_Exp_Comparison.pdf'
plt.savefig(filename_png, dpi=300)
plt.savefig(filename_pdf)

print(f"完了！シミュレーションと実機の比較グラフを保存しました:\n  - {filename_png}\n  - {filename_pdf}")
plt.show()