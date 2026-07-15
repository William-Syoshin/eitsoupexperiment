# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from Controls.PIDcontrol import PIDController
from Controls.AdaptiveControl import STRController

# ==============================================================================
# 1. 共通定数・シミュレーション関数（軌跡取得用）
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
# 2. 実機データ（Experiment）の入力エリア
# ==============================================================================
def extend_to_24(data_list):
    """データが24個未満の場合、最後の要素をコピーして24ステップに引き延ばす"""
    if not data_list:
        return [0.0] * 24
    if len(data_list) >= 24:
        return data_list[:24]
    return data_list + [data_list[-1]] * (24 - len(data_list))

# 💡 以下に実測データを入力してください（途中までの配列でもOKです！）

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
exp_raw_conc_pid = [0.356, 0.455544456, 0.55489022, 0.654037886, 0.660495449] # TODO: 実際の配列に差し替え

# --- STR の実機データ ---
miso_str_raw_conc = [0.270914512, 0.338354573, 0.418397606, 0.507443972, 0.594943023, 0.685628138, 0.771113971, 0.833399432, 0.834334036, 0.835270461]
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
Y_LIM_EC   = (5, 23)      # EC
Y_LIM_TEMP = (45, 55)     # 温度
Y_LIM_SALT = (0, 0.6)     # 塩のスパイク
Y_LIM_CONC = (0, 1.6)     # 濃度

# ==============================================================================
# 4. グラフ描画（1カラム幅・6行1列）
# ==============================================================================
steps = list(range(1, 25))
steps_arr = np.array(steps)

# 💡 データ構造を「上3つがSim、下3つがExp」の縦1列に組み替える
grid_data = [
    # --- Simulation (Row 0, 1, 2) ---
    {'cond': (sim_cond_ol, sim_cond_pid, sim_cond_str), 
     'temp': sim_temp_str, 
     'salt': (sim_salt_ol, sim_salt_pid, sim_salt_str), 
     'conc': (sim_conc_ol, sim_conc_pid, sim_conc_str),
     'title': "Simulation (Miso+Tofu)"},
    
    # --- Experiment (Row 3, 4, 5) ---
    {'cond': (exp_cond_ol, exp_cond_pid, exp_cond_str), 
     'temp': exp_temp_str, 
     'salt': (exp_salt_ol, exp_salt_pid, exp_salt_str), 
     'conc': (exp_conc_ol, exp_conc_pid, exp_conc_str),
     'title': "Experiment (Miso+Tofu)"}
]

plt.rcParams['font.family'] = 'Times New Roman'
# 💡 フォントサイズを9に設定（潰れないギリギリの可読性）
plt.rcParams['font.size'] = 9
plt.rcParams['axes.titlesize'] = 10
plt.rcParams['axes.labelsize'] = 9

# 💡 論文の片側カラムぴったり（8.0cm）に指定
fig_width_in = 8.0 / 2.54
fig_height_in = 18.0 / 2.54

fig, axes = plt.subplots(6, 1, figsize=(fig_width_in, fig_height_in), 
                         gridspec_kw={'height_ratios': [1, 1, 1.2, 1, 1, 1.2], 'hspace': 0.5})

colors = {'OL': '#7f7f7f', 'PID': '#1f77b4', 'STR': '#ce0000', 'Temp': '#ff7f0e'}
styles = {
    'OL':  dict(color=colors['OL'],  linestyle='-',  linewidth=1.5, alpha=0.8),
    'PID': dict(color=colors['PID'], linestyle='--', linewidth=1.8),
    'STR': dict(color=colors['STR'], linestyle='-',  linewidth=2.2),
}

for i in range(2): # 0: Sim, 1: Exp
    data = grid_data[i]
    row_offset = i * 3
    
    # --- 1段目: 伝導率 と 温度 ---
    ax0 = axes[row_offset]
    ax0.set_title(data['title'], fontweight='bold', pad=8)
    
    ax0.plot(steps, data['cond'][0], **styles['OL'])
    ax0.plot(steps, data['cond'][1], **styles['PID'])
    ax0.plot(steps, data['cond'][2], **styles['STR'])
    
    ax0.set_ylim(Y_LIM_EC)
    ax0.set_xticks([1, 6, 12, 18, 24])
    ax0.set_xticklabels([])
    ax0.grid(alpha=0.3)
    ax0.set_ylabel(r"EC $\sigma$" "\n" "[mS/cm]") 
    
    ax0_twin = ax0.twinx()
    ax0_twin.fill_between(steps, 40, data['temp'], color=colors['Temp'], alpha=0.15)
    ax0_twin.plot(steps, data['temp'], color=colors['Temp'], linestyle='-', linewidth=1.5, alpha=0.5)
    ax0_twin.set_ylim(Y_LIM_TEMP)
    ax0_twin.set_ylabel("Temp." "\n" r"[$^\circ$C]")
        
    # --- 2段目: 塩の追加量（極太スパイク） ---
    ax1 = axes[row_offset + 1]
    
    bar_width = 0.8
    ax1.bar(steps_arr, data['salt'][0], width=bar_width, color=colors['OL'], alpha=0.4, zorder=1)
    ax1.bar(steps_arr, data['salt'][1], width=bar_width*0.7, color=colors['PID'], alpha=0.8, zorder=2)
    ax1.bar(steps_arr, data['salt'][2], width=bar_width*0.4, color=colors['STR'], alpha=1.0, zorder=3)
    
    ax1.set_ylim(Y_LIM_SALT)
    ax1.set_yticks([0, 2, 4, 6])
    ax1.set_xticks([1, 6, 12, 18, 24])
    ax1.set_xticklabels([])
    ax1.grid(alpha=0.3)
    ax1.set_ylabel("Added" "\n" "Salt [g]")

    # --- 3段目: 最終濃度 ---
    ax2 = axes[row_offset + 2]
    ax2.axhline(1.0, color='black', linestyle=':', alpha=0.6)
    
    ax2.plot(steps, data['conc'][0], **styles['OL'])
    ax2.plot(steps, data['conc'][1], **styles['PID'])
    ax2.plot(steps, data['conc'][2], **styles['STR'])
    
    ax2.set_ylim(Y_LIM_CONC)
    ax2.set_yticks([0, 0.5, 1.0, 1.5])
    ax2.set_xticks([1, 6, 12, 18, 24])
    ax2.grid(alpha=0.3)
    
    ax2.set_xlabel("Step", labelpad=4)
    ax2.set_ylabel("Conc." "\n" "[%]")

fig.align_ylabels(axes)

# ==============================================================================
# 5. 凡例とレイアウト調整
# ==============================================================================
# 💡 8.0cm幅に合わせて、左右の余白比率を最適化（ラベルが見切れないようにleftを22%に設定）
plt.subplots_adjust(left=0.22, right=0.86, top=0.95, bottom=0.13)

legend_elements = [
    mlines.Line2D([0], [0], color='black', linestyle=':', alpha=0.6, label='Target (1.0%)'),
    mlines.Line2D([0], [0], color=colors['OL'], linestyle='-', linewidth=1.5, alpha=0.8, label='Open-Loop'),
    mlines.Line2D([0], [0], color=colors['PID'], linestyle='--', linewidth=1.8, label='Fixed PID'),
    mlines.Line2D([0], [0], color=colors['STR'], linestyle='-', linewidth=2.2, label='STR (Adaptive)'),
    mpatches.Patch(color=colors['Temp'], alpha=0.3, label='Temperature')
]

# 💡 凡例がはみ出さないように、フォントサイズと文字間隔(columnspacing)を縮小
fig.legend(handles=legend_elements, loc='lower center', bbox_to_anchor=(0.5, 0.01), ncol=2, fontsize=8.5, frameon=False, columnspacing=0.8)

filename_png = 'Sim_Exp_Comparison_1Column.png'
filename_pdf = 'Sim_Exp_Comparison_1Column.pdf'
plt.savefig(filename_png, dpi=300)
plt.savefig(filename_pdf)

print(f"完了！8.0cm幅（1カラム）の比較グラフを保存しました:\n  - {filename_png}\n  - {filename_pdf}")
plt.show()