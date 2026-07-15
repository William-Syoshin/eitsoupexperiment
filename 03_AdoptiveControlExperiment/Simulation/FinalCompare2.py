import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
from matplotlib.legend_handler import HandlerBase  # 💡 ここに部品を追加しました！
from Controls.PIDcontrol import PIDController
from Controls.AdaptiveControl import STRController

# ==========================================
# 0. 共通定数・プラント・シミュレーション関数
# ==========================================
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
    def __init__(self, m_water, initial_salt_pct, true_alpha, true_beta):
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
    
    @property
    def get_ec(self):
        sigma_base = self.true_alpha * self.true_concentration + self.true_beta
        true_ec = sigma_base * (1.0 + TEMP_COEFF * (T_CONST - T_BASE))
        noisy_ec = true_ec * np.random.normal(loc=1.0, scale=0.01)
        return noisy_ec

def run_simulation(m_water, init_salt_pct, true_alpha, true_beta, mode="STR"):
    plant = FastSoupPlant(m_water, init_salt_pct, true_alpha, true_beta)
    str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, lam=1.0)
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD, output_min=0.0, output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)
    
    total_salt, zero_salt_count = 0.0, 0
    alpha_h, beta_h = ALPHA_NOMINAL, BETA_NOMINAL
    prev_alpha, prev_beta = ALPHA_NOMINAL, BETA_NOMINAL
    C_init_virtual = 0.0
    
    if mode == "OL":
        initial_ol_salt = 6.0
        plant.added_salt += initial_ol_salt
        plant.salt_mass += initial_ol_salt
        total_salt += initial_ol_salt
    
    for step in range(MAX_STEPS):
        sigma_comp = plant.get_ec / (1.0 + TEMP_COEFF * (T_CONST - T_BASE))
        salt_g = 0.0
        
        if mode == "OL":
            break # OLは最初に一気に入れて終わりなのでこれ以上計算しない
            
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
            str_unit.theta[0, 0], str_unit.theta[1, 0] = alpha_h, beta_h
            
            adapt_ratio = min(max(ALPHA_NOMINAL / alpha_h, 0.3), 3.0)
            pid.Kp, pid.Ki, pid.Kd = KP * adapt_ratio, KI * adapt_ratio, KD * adapt_ratio
            
            C_true_abs = C_init_virtual if total_salt < 0.01 else (sigma_comp - beta_h) / alpha_h
            salt_g = pid.compute(C_TARGET - C_true_abs, SALT_INTERVAL) * SALT_INTERVAL
            
        salt_g = min(salt_g, SALT_MAX_PER_STEP)
        if salt_g >= 0.01:
            plant.add_salt(salt_g)
            total_salt += salt_g
            zero_salt_count = 0
        else:
            zero_salt_count += 1
            if zero_salt_count >= 2: break

    # 🌟 エラーは絶対値に変換して比較
    return abs((plant.true_concentration - C_TARGET) / C_TARGET) * 100.0

# ==========================================
# 1. データの準備
# ==========================================
labels = ['Pure\nWater', 'Miso\nSoup', 'Miso Soup\n+ Tofu']
x = np.arange(len(labels))

# --- 実機データ (Experiment: いただいた最新データを代入) ---
exp_err_ol  = np.array([0.0, 0.277227723, 0.529644269]) * 100
exp_err_pid = np.array([0.0150396775, 0.261042306, 0.347480104]) * 100
exp_err_str = np.array([0.041146102, 0.171022024, 0.089823975]) * 100

# --- シミュレーションデータ (Simulation: 10回平均を自動計算) ---
N_TRIALS = 10

# 3つの環境の物理パラメータ [m_water, init_salt_pct, true_alpha, true_beta]
env_params = [
    [600.0, 0.000, 8.1013, 0.1550],  # Pure Water
    [600.0, 0.290, 9.1819, 1.1606],  # Miso Soup
    [500.0, 0.348, 8.5359, 1.1373]   # Miso Soup + Tofu
]

sim_err_ol = []
sim_err_pid = []
sim_err_str = []

print(f"シミュレーションを実行中... (各環境 {N_TRIALS}回平均)")
for params in env_params:
    err_ol_list, err_pid_list, err_str_list = [], [], []
    for _ in range(N_TRIALS):
        err_ol_list.append(run_simulation(*params, mode="OL"))
        err_pid_list.append(run_simulation(*params, mode="PID"))
        err_str_list.append(run_simulation(*params, mode="STR"))
    
    sim_err_ol.append(np.mean(err_ol_list))
    sim_err_pid.append(np.mean(err_pid_list))
    sim_err_str.append(np.mean(err_str_list))

sim_err_ol = np.array(sim_err_ol)
sim_err_pid = np.array(sim_err_pid)
sim_err_str = np.array(sim_err_str)

# ==========================================
# 2. グラフの設定 (幅8cm, 1カラム用)
# ==========================================
fig_width_inch = 8.0 / 2.54
fig_height_inch = 8.0 / 2.54 

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix' 
plt.rcParams['font.size'] = 9   

fig, ax = plt.subplots(figsize=(fig_width_inch, fig_height_inch))

# --- デザイン設定 ---
color_ol  = '#7f7f7f'   # グレー (Open-Loop)
color_pid = '#1f77b4'   # 青 (PID Control)
color_str = '#c00000'   # 赤 (STR Adaptive)

lw_exp = 1.8            
ms = 7                  
lw_sim = 1.5            
alpha_band = 0.15       
band_width = 4.0        

# ==========================================
# 3. 描画 (1) シミュレーション (背景の帯とトレンド線)
# ==========================================
ax.fill_between(x, sim_err_ol - band_width, sim_err_ol + band_width, color=color_ol, alpha=alpha_band, zorder=1)
ax.fill_between(x, sim_err_pid - band_width, sim_err_pid + band_width, color=color_pid, alpha=alpha_band, zorder=1)
ax.fill_between(x, sim_err_str - band_width, sim_err_str + band_width, color=color_str, alpha=alpha_band, zorder=1)

ax.plot(x, sim_err_ol, color=color_ol, linestyle='--', linewidth=lw_sim, alpha=0.7, zorder=2)
ax.plot(x, sim_err_pid, color=color_pid, linestyle='--', linewidth=lw_sim, alpha=0.7, zorder=2)
ax.plot(x, sim_err_str, color=color_str, linestyle='--', linewidth=lw_sim, alpha=0.7, zorder=2)

# ==========================================
# 4. 描画 (2) 実機実験 (最前面の実線＋マーカー)
# ==========================================
ax.plot(x, exp_err_ol, color=color_ol, marker='o', markersize=ms, linestyle='-', linewidth=lw_exp, zorder=4)
ax.plot(x, exp_err_pid, color=color_pid, marker='s', markersize=ms, linestyle='-', linewidth=lw_exp, zorder=4)
ax.plot(x, exp_err_str, color=color_str, marker='D', markersize=ms+1, linestyle='-', linewidth=lw_exp, zorder=4)

# ==========================================
# 5. 実機実験の数値を直書きする処理
# ==========================================
offset_ol  = [(0, 15), (0, 22), (0, 10)]      
offset_pid = [(0, 21), (0, 15), (0, 10)]     
offset_str = [(0, 25), (0, -15), (0, -15)]


def add_value_labels_smart(x_coords, y_coords, text_color, offsets):
    for i, (x_val, y_val) in enumerate(zip(x_coords, y_coords)):
        ax.annotate(f"{y_val:.1f}%",
                    xy=(x_val, y_val),
                    xytext=offsets[i],
                    textcoords="offset points",
                    ha='center', va='center',
                    fontsize=8.5, fontweight='bold', color=text_color, zorder=5)

add_value_labels_smart(x, exp_err_ol, '#4d4d4d', offset_ol)
add_value_labels_smart(x, exp_err_pid, '#124870', offset_pid)
add_value_labels_smart(x, exp_err_str, '#8b0000', offset_str)

# ==========================================
# 6. 装飾とレイアウト調整
# ==========================================
ax.set_ylabel('Steady-State\nSalinity Error [%]',y=0.5)
ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=9)

ax.set_xlim(-0.25, 2.25)
ax.set_ylim(-5, 65) 
ax.grid(True, axis='y', linestyle=':', alpha=0.6)

# 💡 矢印とテキストのY座標を下げて、2行のスープ名ラベルと被らないようにしました
ax.annotate('', xy=(1.0, -0.3), xycoords='axes fraction', xytext=(0.0, -0.3),
            arrowprops=dict(arrowstyle="->", color="black", lw=1.2))
ax.text(0.5, -0.38, 'Increasing Soup Complexity', transform=ax.transAxes, 
        ha='center', va='center', fontsize=9, fontweight='bold')

# ==========================================
# 🌟 超絶カスタマイズ：自作の凡例アイコンを定義
# ==========================================
class ExpHandler(HandlerBase):
    def create_artists(self, legend, orig_handle, xdescent, ydescent, width, height, fontsize, trans):
        line = plt.Line2D([xdescent, xdescent + width], [ydescent + height/2., ydescent + height/2.], 
                          color='black', lw=1.8, transform=trans)
        m1 = plt.Line2D([xdescent + width*0.15], [ydescent + height/2.], 
                        marker='o', color='black', markersize=5.5, linestyle='None', transform=trans)
        m2 = plt.Line2D([xdescent + width*0.5],  [ydescent + height/2.], 
                        marker='s', color='black', markersize=5.5, linestyle='None', transform=trans)
        m3 = plt.Line2D([xdescent + width*0.85], [ydescent + height/2.], 
                        marker='D', color='black', markersize=6, linestyle='None', transform=trans)
        return [line, m1, m2, m3]

class SimHandler(HandlerBase):
    def create_artists(self, legend, orig_handle, xdescent, ydescent, width, height, fontsize, trans):
        patch = plt.Rectangle((xdescent, ydescent - height*0.1), width, height*1.2, 
                              facecolor='black', alpha=0.15, transform=trans)
        line = plt.Line2D([xdescent, xdescent + width], [ydescent + height/2., ydescent + height/2.], 
                          color='black', linestyle='--', lw=1.5, transform=trans)
        return [patch, line]

class ExpDummy: pass
class SimDummy: pass

# --- 凡例の設定 ---
legend_ctrl = [
    Line2D([0], [0], color=color_ol, marker='o', linestyle='-', lw=lw_exp, markersize=ms, label='Open-Loop'),
    Line2D([0], [0], color=color_pid, marker='s', linestyle='-', lw=lw_exp, markersize=ms, label='PID Control'),
    Line2D([0], [0], color=color_str, marker='D', linestyle='-', lw=lw_exp, markersize=ms+1, label='Proposed Framework')
]

leg1 = ax.legend(handles=legend_ctrl, loc='upper center', bbox_to_anchor=(0.5, -0.47), 
                 ncol=3, frameon=False, fontsize=8, columnspacing=0.8, handletextpad=0.4)

leg2 = ax.legend(handles=[ExpDummy(), SimDummy()], labels=['Experiment', 'Simulation'],
                 loc='upper center', bbox_to_anchor=(0.5, -0.57), 
                 ncol=2, frameon=False, fontsize=8, columnspacing=2.0, handletextpad=0.6,
                 handlelength=3.5, 
                 handler_map={ExpDummy: ExpHandler(), SimDummy: SimHandler()})

ax.add_artist(leg1) 

plt.subplots_adjust(bottom=0.50, left=0.18, right=0.95, top=0.95)

filename_png = 'Fig10_line_SimExp_8cm.png'
filename_pdf = 'Fig10_line_SimExp_8cm.pdf'
plt.savefig(filename_png, dpi=600, bbox_inches='tight', 
            bbox_extra_artists=(leg1, leg2), pad_inches=0.05)
plt.savefig(filename_pdf, bbox_inches='tight', 
            bbox_extra_artists=(leg1, leg2), pad_inches=0.05)

print(f"完了！8cm幅に最適化されたグラフを保存しました:\n  - {filename_png}\n  - {filename_pdf}")
plt.show()