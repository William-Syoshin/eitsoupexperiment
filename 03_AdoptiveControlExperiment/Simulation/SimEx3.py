# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from Controls.PIDcontrol import PIDController
from Controls.AdaptiveControl import STRController

# ==============================================================================
# 1. 共通定数・シミュレーション関数（省略なし・変更なし）
# ==============================================================================
MAX_STEPS = 24
SALT_INTERVAL = 30.0
C_TARGET = 1.0
SALT_MAX_PER_STEP = 0.5
M_TOTAL_ASSUMED = 600.0  

ALPHA_NOMINAL = 8.1013
BETA_NOMINAL  = 0.1550
TEMP_COEFF    = 0.02
T_BASE        = 24.6

ALPHA_MIN, ALPHA_MAX = ALPHA_NOMINAL / 3.0, ALPHA_NOMINAL * 3.0
BETA_MIN, BETA_MAX = 0.0, 5.0
D_ALPHA_MAX, D_BETA_MAX = 0.5, 0.2
DEADBAND = 0.02
KP, KI, KD = 0.1, 0.00, 0.0
T_CONST = 50.0

class FastSoupPlant:
    def __init__(self, m_water=500.0, initial_salt_pct=0.348):
        self.m_liquid_base = m_water 
        self.salt_mass = self.m_liquid_base * (initial_salt_pct / 100.0)
        self.added_salt = 0.0
        self.true_alpha = 8.5359  
        self.true_beta = 1.1373
        
    def add_salt(self, target_salt_g):
        actual_salt_g = target_salt_g * np.random.normal(loc=1.0, scale=0.05)
        self.added_salt += actual_salt_g
        self.salt_mass += actual_salt_g
        
    @property
    def true_concentration(self):
        return (self.salt_mass / (self.m_liquid_base + self.added_salt)) * 100.0
    
    def get_ec(self, current_temp):
        sigma_base = self.true_alpha * self.true_concentration + self.true_beta
        true_ec = sigma_base * (1.0 + TEMP_COEFF * (current_temp - T_BASE))
        noisy_ec = true_ec * np.random.normal(loc=1.0, scale=0.01)
        return noisy_ec

def run_simulation_trace(mode="STR"):
    plant = FastSoupPlant(m_water=500.0, initial_salt_pct=0.348)
    str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, lam=1.0)
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD, output_min=0.0, output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)
    
    total_salt = 0.0
    alpha_h, beta_h = ALPHA_NOMINAL, BETA_NOMINAL
    prev_alpha, prev_beta = ALPHA_NOMINAL, BETA_NOMINAL
    C_init_virtual = 0.0
    
    zero_salt_count = 0
    is_cooking_finished = False
    
    target_temp = 50.0 
    history_cond, history_temp, history_salt, history_conc = [], [], [], []
    
    if mode == "OL":
        initial_ol_salt = 6.0
        plant.added_salt += initial_ol_salt
        plant.salt_mass += initial_ol_salt
        total_salt += initial_ol_salt
    
    for step in range(MAX_STEPS):
        current_temp = target_temp + np.random.normal(0, 0.5)
        raw_ec = plant.get_ec(current_temp)
        sigma_comp = raw_ec / (1.0 + TEMP_COEFF * (current_temp - T_BASE))
        
        history_temp.append(current_temp)
        history_cond.append(raw_ec)
        history_conc.append(plant.true_concentration)
        
        salt_g = 0.0
        
        if mode == "OL":
            history_salt.append(6.0 if step == 0 else 0.0)
            continue  
            
        if is_cooking_finished:
            history_salt.append(0.0)
            continue
                
        elif mode == "PID":
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
        
        if salt_g < 0.01:
            zero_salt_count += 1
            if zero_salt_count >= 2:
                is_cooking_finished = True
        else:
            zero_salt_count = 0  
            
        history_salt.append(salt_g)
        
        if salt_g >= 0.01:
            plant.add_salt(salt_g)
            total_salt += salt_g

    return history_cond, history_temp, history_salt, history_conc

sim_cond_ol,  sim_temp_ol,  sim_salt_ol,  sim_conc_ol  = run_simulation_trace(mode="OL")
sim_cond_pid, sim_temp_pid, sim_salt_pid, sim_conc_pid = run_simulation_trace(mode="PID")
sim_cond_str, sim_temp_str, sim_salt_str, sim_conc_str = run_simulation_trace(mode="STR")

# ==============================================================================
# 2. 実機データ入力（変更なし）
# ==============================================================================
def extend_to_24(data_list):
    if not data_list:
        return [0.0] * 24
    if len(data_list) >= 24:
        return data_list[:24]
    return data_list + [data_list[-1]] * (24 - len(data_list))

# --- Open-Loop (OL) の実機データ ---
exp_raw_cond_ol = [21.7,
21.7,
21.6,
21.6,
21.7,
21.7,
21.6,
21.6,
21.5,
21.5,
21.5,
21.5,
21.5,
21.5,
21.5,
21.5,
21.4,
21.4,
21.7,
21.7,
21.7,
21.7,
21.3,
21.3] # TODO: 実際の配列に差し替え
exp_raw_temp_ol = [48.83,
48.83,
47.76,
47.76,
47.79,
47.79,
48.99,
48.99,
49.5,
49.5,
49.54,
49.54,
49.4,
49.4,
49.33,
49.33,
49.23,
49.23,
49.13,
49.13,
49.02,
49.02,
48.89,
48.89] # TODO: 実際の配列に差し替え
exp_raw_salt_ol = [6.0, 0.0] # TODO: 実際の配列に差し替え
exp_raw_conc_ol = [1.529644269] # TODO: 実際の配列に差し替え

# --- PID の実機データ ---
exp_raw_cond_pid = [7.63,
8.86,
10.59,
12.24,
12.31,
12.45,
12.08,
12.34,
12.34,
12.37,
12.37,
12.37,
12.37,
12.38,
12.4,
12.36,
12.38,
12.38,
12.38,
12.36,
12.39,
12.39,
12.39,
12.39] # TODO: 実際の配列に差し替え
exp_raw_temp_pid = [47.29,
46.78,
46.27,
44.94,
44.33,
45.62,
48.07,
48.17,
48.07,
48.04,
47.97,
47.87,
47.73,
47.66,
47.56,
47.49,
47.53,
47.49,
47.49,
47.46,
47.39,
47.36,
47.25,
47.25] # TODO: 実際の配列に差し替え
exp_raw_salt_pid = [0.5, 0.5, 0.5, 0.0326, 0.0] # TODO: 実際の配列に差し替え
exp_raw_conc_pid = [0.348,
0.447552448,
0.546906188,
0.646061815,
0.646061815,
0.646061815,
0.646061815,
0.652519896] # TODO: 実際の配列に差し替え

# --- STR の実機データ ---
miso_str_raw_conc = [0.348,
0.447552448,
0.546906188,
0.646061815,
0.74501992,
0.843781095,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206,
0.910176206]
exp_raw_cond_str = [6.62,
7.44,
8.54,
9.69,
10.93,
11.39,
13.02,
13.04,
13.04,
13,
13.07,
13.08,
13.1,
13.1,
13.1,
13.1,
13.1,
13.2,
13.14,
13.14,
13.38,
13.38,
13.38,
13.38]      # TODO: 実際の配列に差し替え
exp_raw_temp_str = [46.98,
48.72,
48.65,
48.17,
47.39,
47.05,
46.67,
46.47,
46.4,
46.4,
46.37,
46.33,
46.78,
47.08,
47.46,
47.56,
47.59,
47.76,
47.87,
47.7,
47.42,
47.8,
47.97,
48.07]      # TODO: 実際の配列に差し替え
exp_raw_salt_str = [0.5, 0.5, 0.5, 0.5, 0.5, 0.3367, 0.0]      # TODO: 実際の配列に差し替え
exp_raw_conc_str = miso_str_raw_conc        # ← いただいたデータをそのまま入れています

# --- 自動延長処理（触らなくてOK） ---
exp_cond_ol,  exp_temp_ol,  exp_salt_ol,  exp_conc_ol  = map(extend_to_24, [exp_raw_cond_ol, exp_raw_temp_ol, exp_raw_salt_ol, exp_raw_conc_ol])
exp_cond_pid, exp_temp_pid, exp_salt_pid, exp_conc_pid = map(extend_to_24, [exp_raw_cond_pid, exp_raw_temp_pid, exp_raw_salt_pid, exp_raw_conc_pid])
exp_cond_str, exp_temp_str, exp_salt_str, exp_conc_str = map(extend_to_24, [exp_raw_cond_str, exp_raw_temp_str, exp_raw_salt_str, exp_raw_conc_str])

# ==============================================================================
# 3. Y軸の表示範囲設定
# ==============================================================================
Y_LIM_EC   = (5, 23)      
Y_LIM_TEMP = (45, 55)     
Y_LIM_SALT = (0, 0.6)     
Y_LIM_CONC = (0, 1.7)     

# ==============================================================================
# 4. グラフ描画（本体縦縮小＆目盛りサイズ6版）
# ==============================================================================
steps = list(range(1, 25))
steps_arr = np.array(steps)

grid_cond = [(sim_cond_ol, sim_cond_pid, sim_cond_str), (exp_cond_ol, exp_cond_pid, exp_cond_str)]
grid_temp = [(sim_temp_ol, sim_temp_pid, sim_temp_str), (exp_temp_ol, exp_temp_pid, exp_temp_str)]
grid_salt = [(sim_salt_ol, sim_salt_pid, sim_salt_str), (exp_salt_ol, exp_salt_pid, exp_salt_str)]
grid_conc = [(sim_conc_ol, sim_conc_pid, sim_conc_str), (exp_conc_ol, exp_conc_pid, exp_conc_str)]

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 8
plt.rcParams['axes.titlesize'] = 10    
plt.rcParams['axes.labelsize'] = 9     

# 💡 要求通り、目盛りの数字（ラベル）の大きさを「6」に固定
plt.rcParams['xtick.labelsize'] = 6   
plt.rcParams['ytick.labelsize'] = 6

# 全体の枠（高さ）を 8.5cm までさらに下げます（幅15cm固定）
fig, axes = plt.subplots(2, 3, figsize=(15 / 2.54, 8.5 / 2.54))

colors = {'OL': '#7f7f7f', 'PID': '#1f77b4', 'STR': '#ce0000', 'Temp': '#ff7f0e'}
styles = {
    'OL':  dict(color=colors['OL'],  linestyle='-',  linewidth=1.5, alpha=0.8),
    'PID': dict(color=colors['PID'], linestyle='--', linewidth=1.5),
    'STR': dict(color=colors['STR'], linestyle='-',  linewidth=2.0),
}

col_titles = ["Conductivity", "Added Salt", "Salinity"]
row_labels = ["Simulation", "Experiment"]

for row in range(2):
    # ==========================
    # 1列目: 伝導率 と 温度
    # ==========================
    ax0 = axes[row, 0]
    if row == 0: ax0.set_title(col_titles[0], fontweight='bold', pad=6)
    
    ax0.plot(steps, grid_cond[row][0], **styles['OL'])
    ax0.plot(steps, grid_cond[row][1], **styles['PID'])
    ax0.plot(steps, grid_cond[row][2], **styles['STR'])
    
    ax0.set_ylim(Y_LIM_EC)
    ax0.set_xticks([1, 6, 12, 18, 24])
    if row == 0: ax0.set_xticklabels([])
    else: ax0.set_xlabel("Step")
    ax0.grid(alpha=0.3)
    ax0.set_ylabel(r"$\sigma$ [mS/cm]", labelpad=2)
    
    # Simulation / Experiment の行ラベル（大きさ10）
    ax0.annotate(row_labels[row], xy=(0, 0.5), xytext=(-0.4, 0.5),
                 xycoords='axes fraction', textcoords='axes fraction',
                 fontsize=10, fontweight='bold', ha='center', va='center', rotation=90)
    
    ax0_twin = ax0.twinx()
    ax0_twin.fill_between(steps, 40, grid_temp[row][2], color=colors['Temp'], alpha=0.15)
    ax0_twin.plot(steps, grid_temp[row][2], color=colors['Temp'], linestyle='-', linewidth=1.0, alpha=0.5)
    ax0_twin.set_ylim(Y_LIM_TEMP)
    ax0_twin.set_ylabel(r"$T$ [$^\circ$C]", color=colors['Temp'], labelpad=4)
    ax0_twin.tick_params(axis='y', colors=colors['Temp'], labelsize=6) # 💡右目盛りも6に
    
    # ==========================
    # 2列目: 塩の追加量
    # ==========================
    ax1 = axes[row, 1]
    if row == 0: ax1.set_title(col_titles[1], fontweight='bold', pad=6)
    
    bar_width = 0.4
    ax1.bar(steps_arr - bar_width, grid_salt[row][0], width=bar_width, color=colors['OL'], alpha=0.7, zorder=3)
    ax1.bar(steps_arr,             grid_salt[row][1], width=bar_width, color=colors['PID'], alpha=0.9, zorder=3)
    ax1.bar(steps_arr + bar_width, grid_salt[row][2], width=bar_width, color=colors['STR'], alpha=1.0, zorder=3)
    
    if grid_salt[row][0][0] > 0.6:
        ax1.annotate('6.0g', xy=(1 - bar_width, 0.55), xytext=(30, -3),
                     textcoords="offset points", ha='left', va='center', 
                     color=colors['OL'], fontsize=8, fontweight='bold', zorder=4,
                     arrowprops=dict(arrowstyle="->", color=colors['OL'], linewidth=1.0))

    ax1.set_ylim(Y_LIM_SALT)
    ax1.set_yticks([0, 0.3, 0.6])
    ax1.set_xticks([1, 6, 12, 18, 24])
    if row == 0: ax1.set_xticklabels([])
    else: ax1.set_xlabel("Step")
    ax1.grid(alpha=0.3)
    ax1.set_ylabel(r"$u$ [g]", labelpad=2)

    # ==========================
    # 3列目: 最終濃度
    # ==========================
    ax2 = axes[row, 2]
    if row == 0: ax2.set_title(col_titles[2], fontweight='bold', pad=6)
    
    ax2.axhline(1.0, color='black', linestyle=':', linewidth=1.2, alpha=0.6)
    
    ax2.plot(steps, grid_conc[row][0], **styles['OL'])
    ax2.plot(steps, grid_conc[row][1], **styles['PID'])
    ax2.plot(steps, grid_conc[row][2], **styles['STR'])
    
    ax2.set_ylim(Y_LIM_CONC)
    ax2.set_yticks([0, 0.5, 1.0, 1.5])
    ax2.set_xticks([1, 6, 12, 18, 24])
    if row == 0: ax2.set_xticklabels([])
    else: ax2.set_xlabel("Step")
    ax2.grid(alpha=0.3)
    ax2.set_ylabel(r"$C$ [%]", labelpad=2)

# ==============================================================================
# 5. グラフサイズ・位置の完全固定調整（独立調整）
# ==============================================================================
# 💡 グラフ自体の大きさをここで完全に数値指定します
w0, w1, w2 = 0.18, 0.18, 0.24  # 各グラフの横幅（3列目は広め）
h = 0.24                       # 💡【解決】グラフ本体の縦の大きさをここで強制的に小さく固定！

# 💡 各グラフの左端の開始位置（X座標）をコントロールして隙間を独立調整
x0 = 0.04
x1 = x0 + w0 + 0.15  # 💡1個目と2個目の間：ここを広げる（0.12の隙間）
x2 = x1 + w1 + 0.08 # 💡2個目と3個目の間：ここは詰める（0.05の隙間）

# 💡 各行の下端の開始位置（Y座標）をコントロールして上下の隙間を調整
y1 = 0.26            # 下段（Experiment）の位置
y0 = y1 + h + 0.07   # 上段（Simulation）の位置（0.07の隙間＝空きすぎを解消）

# 確定した配置を一斉に適用
for col, (x, w) in enumerate([(x0, w0), (x1, w1), (x2, w2)]):
    axes[0, col].set_position([x, y0, w, h])
    axes[1, col].set_position([x, y1, w, h])

# --- 凡例の設定（位置もグラフの縮小に合わせて微調整） ---
legend_controls = [
    mlines.Line2D([0], [0], color=colors['OL'], linestyle='-', linewidth=1.5, alpha=0.8, label='Open-Loop'),
    mlines.Line2D([0], [0], color=colors['PID'], linestyle='--', linewidth=1.5, label='PID Control'),
    mlines.Line2D([0], [0], color=colors['STR'], linestyle='-', linewidth=2.0, label='Proposed Framework')
]
legend_others = [
    mpatches.Patch(color=colors['Temp'], alpha=0.3, label='Temperature'),
    mlines.Line2D([0], [0], color='black', linestyle=':', linewidth=1.5, alpha=0.6, label='Target (1.0%)')
]

leg1 = fig.legend(handles=legend_controls, loc='lower center', bbox_to_anchor=(0.41, 0.07), ncol=3, fontsize=9, frameon=False, columnspacing=1.5)
leg2 = fig.legend(handles=legend_others,   loc='lower center', bbox_to_anchor=(0.41, 0.025), ncol=2, fontsize=9, frameon=False, columnspacing=3.0)

fig.add_artist(leg1) 

filename_png = '3.png'
filename_pdf = '3.pdf'
plt.savefig(filename_png, dpi=300, bbox_inches='tight')
plt.savefig(filename_pdf, bbox_inches='tight')
plt.show()