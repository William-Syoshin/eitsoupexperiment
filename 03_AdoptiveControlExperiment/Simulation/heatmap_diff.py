# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController

# ==========================================
# 共通定数・プラント・シミュレーション関数
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
D_ALPHA_MAX, D_BETA_MAX =  0.5, 0.2
DEADBAND = 0.02
KP, KI, KD = 0.1, 0.00, 0.0
T_CONST = 50.0

class FastSoupPlant:
    def __init__(self, m_water=500.0, initial_salt_pct=0.0, true_alpha=8.1, true_beta=0.15):
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

def run_simulation(true_alpha, true_beta, init_salt_pct=0.0, mode="STR"):
    plant = FastSoupPlant(m_water=500.0, initial_salt_pct=init_salt_pct, 
                          true_alpha=true_alpha, true_beta=true_beta)
    str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, lam=1.0)
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD, output_min=0.0, output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)
    
    total_salt, zero_salt_count = 0.0, 0
    alpha_h, beta_h = ALPHA_NOMINAL, BETA_NOMINAL
    prev_alpha, prev_beta = ALPHA_NOMINAL, BETA_NOMINAL
    C_init_virtual = 0.0
    
    for step in range(MAX_STEPS):
        sigma_comp = plant.get_ec / (1.0 + TEMP_COEFF * (T_CONST - T_BASE))
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

    return ((plant.true_concentration - C_TARGET) / C_TARGET) * 100.0

# ==========================================
# 4. 1行3列ブロック型ヒートマップ描画
# ==========================================
if __name__ == "__main__":
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 8  
    
    INIT_SALT = 1.78 / (500.0 + 1.78) * 100
    N_TRIALS = 10
    
    RESOLUTION_X = 11  
    RESOLUTION_Y = 11  
    
    alpha_range = np.linspace(4.0, 14.0, RESOLUTION_X)
    beta_range = np.linspace(0.5, 2.5, RESOLUTION_Y)
    
    A, B = np.meshgrid(alpha_range, beta_range)
    Z_pid = np.zeros_like(A)
    Z_str = np.zeros_like(A)
    Z_diff = np.zeros_like(A)
    
    print(f"確率論的シミュレーションを実行中... (各マス {N_TRIALS}回平均)")
    for i in range(RESOLUTION_Y):
        for j in range(RESOLUTION_X):
            pid_errors, str_errors = [], []
            for _ in range(N_TRIALS):
                e_pid = run_simulation(A[i, j], B[i, j], init_salt_pct=INIT_SALT, mode="PID")
                e_str = run_simulation(A[i, j], B[i, j], init_salt_pct=INIT_SALT, mode="STR")
                
                pid_errors.append(e_pid)
                str_errors.append(e_str)
                
            Z_pid[i, j] = np.mean(pid_errors)
            Z_str[i, j] = np.mean(str_errors)
            Z_diff[i, j] = np.abs(Z_pid[i, j]) - np.abs(Z_str[i, j])
            
    fig = plt.figure(figsize=(15/2.54, 4.5/2.54), constrained_layout=True)
    
    # 🌟 【縦線追加の裏技】全体を3列に分割：[左グループ(2.05)] -> [黒い線用の隙間(0.06)] -> [右グループ(1.05)]
    gs_main = fig.add_gridspec(1, 3, width_ratios=[2.05, 0.06, 1.05])
    
    # --- 左グループ: (a) PID, (b) STR, エラー用カラーバー ---
    gs_left = gs_main[0].subgridspec(1, 3, width_ratios=[1, 1, 0.05])
    ax_pid = fig.add_subplot(gs_left[0, 0])
    ax_str = fig.add_subplot(gs_left[0, 1])
    cax_err = fig.add_subplot(gs_left[0, 2])
    
    # 🌟 --- 中央ブロック: (b)と(c)を隔てる「黒い縦線」 ---
    ax_line = fig.add_subplot(gs_main[1])
    # 枠線も目盛りもすべて消して「透明なグラフ」にする
    ax_line.spines['top'].set_visible(False)
    ax_line.spines['bottom'].set_visible(False)
    ax_line.spines['left'].set_visible(False)
    ax_line.spines['right'].set_visible(False)
    ax_line.set_xticks([])
    ax_line.set_yticks([])
    # ダミーのタイトルとXラベルを入れて、高さを(a)(b)(c)のマップと完全に1ミリの狂いもなく合わせる！
    ax_line.set_title(' ', fontsize=8, pad=6)
    ax_line.set_xlabel(' ', fontsize=7.5, labelpad=1)
    # 真ん中にスッと黒い縦線を引く
    ax_line.axvline(0.5, ymin=-2, ymax=2 ,color='black', linewidth=1.2,clip_on=False)
    
    # --- 右グループ: (c) 改善度, 改善度用カラーバー ---
    gs_right = gs_main[2].subgridspec(1, 2, width_ratios=[1, 0.05])
    ax_diff = fig.add_subplot(gs_right[0, 0])
    cax_diff = fig.add_subplot(gs_right[0, 1])
    
    axes = [ax_pid, ax_str, ax_diff]
    datas = [Z_pid, Z_str, Z_diff]
    titles = ['(a) PID Control', '(b) Adaptive Tasting Control', '(c) Performance Improvement']
    
    colors_green = [
        (0.0, "#e0e0e0"),  
        (0.5, "#ffffff"),  
        (0.6, "#e0f2f1"),  
        (0.8, "#4db6ac"),  
        (1.0, "#00695c")   
    ]
    custom_green_cmap = LinearSegmentedColormap.from_list("ImprovementGreen", colors_green)
    
    cmaps = ['coolwarm', 'coolwarm', custom_green_cmap]  
    vmins = [-60, -60, -40]
    vmaxs = [60, 60, 40]
    
    real_soups = [(8.5359, 1.1373, 'Miso+Tofu', 'lime')]

    for idx, (ax, Z, title, cmap, vmin, vmax) in enumerate(zip(axes, datas, titles, cmaps, vmins, vmaxs)):
        dx = (alpha_range[1] - alpha_range[0]) / 2.0
        dy = (beta_range[1] - beta_range[0]) / 2.0
        extent = [alpha_range[0]-dx, alpha_range[-1]+dx, beta_range[0]-dy, beta_range[-1]+dy]
        
        c = ax.imshow(Z, origin='lower', cmap=cmap, vmin=vmin, vmax=vmax, extent=extent, aspect='auto')
        ax.set_box_aspect(1)
        
        for i in range(RESOLUTION_Y):
            for j in range(RESOLUTION_X):
                val = Z[i, j]
                text_color = 'white' if abs(val) > (vmax * 0.35) else 'black'
                ax.text(alpha_range[j], beta_range[i], f'{val:.1f}', 
                        ha='center', va='center', color=text_color, fontsize=3.8, alpha=0.9)
        
        for a_val, b_val, name, color in real_soups:
            ax.plot(a_val, b_val, marker='o', markersize=2.2, color=color, markeredgecolor='black', zorder=5)
            ax.annotate(name, xy=(a_val, b_val), xytext=(18, 12), textcoords='offset points',
                        ha='center', va='center', fontsize=6.5, weight='bold', color='black',
                        bbox=dict(boxstyle="round,pad=0.15", facecolor='white', alpha=0.9, edgecolor='gray'),
                        arrowprops=dict(arrowstyle="->", color="black", linewidth=0.6, connectionstyle="arc3,rad=0.1"), zorder=6)
            
        ax.set_title(title, fontsize=8, fontweight='bold', pad=6)
        ax.set_xlabel('True Parameter $\\alpha$', fontsize=7.5, labelpad=1)
        
        if idx == 0:
            ax.set_ylabel('True Parameter $\\beta$', fontsize=7.5, labelpad=2)
            ax.tick_params(axis='y', labelleft=True)   
        else:
            ax.set_ylabel('')                          
            ax.tick_params(axis='y', labelleft=False)  
        
        ax.set_xticks(alpha_range[::2])
        ax.set_yticks(beta_range[::2])
        ax.tick_params(axis='both', which='major', labelsize=7)
    
    # カラーバーの配置
    cbar1 = fig.colorbar(axes[1].images[0], cax=cax_err)
    cbar1.set_label('Final Error [%]', fontsize=7.5, labelpad=2)
    cbar1.ax.tick_params(labelsize=6.5)
    
    cbar2 = fig.colorbar(axes[2].images[0], cax=cax_diff)
    cbar2.set_label('Error Reduction ($|e_{\\mathrm{PID}}| - |e_{\\mathrm{STR}}|$) [%]', fontsize=7.5, labelpad=2)
    cbar2.ax.tick_params(labelsize=6.5)
    
    filename_png = 'Heatmap_diff.png'
    filename_pdf = 'Heatmap_diff.pdf'
    plt.savefig(filename_png, dpi=300, bbox_inches='tight')
    plt.savefig(filename_pdf, bbox_inches='tight')
    
    print(f"完了！セパレーター縦線入りのプロ仕様1行3列画像を保存しました:\n  - {filename_png}\n  - {filename_pdf}")
    plt.show()