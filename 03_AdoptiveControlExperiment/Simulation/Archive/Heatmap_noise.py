# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
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
M_TOTAL_ASSUMED = 600.0  # コントローラが想定するスープ総重量（実機と同様に600gと勘違いさせる）

ALPHA_NOMINAL = 8.1013
BETA_NOMINAL  = 0.1550
TEMP_COEFF    = 0.02
T_BASE        = 24.6

ALPHA_MIN, ALPHA_MAX = ALPHA_NOMINAL / 3.0, ALPHA_NOMINAL * 3.0
BETA_MIN, BETA_MAX = 0.0, 5.0
D_ALPHA_MAX, D_BETA_MAX = 0.5, 0.2
DEADBAND = 0.02
KP, KI, KD = 0.1, 0.005, 0.0
T_CONST = 50.0

class FastSoupPlant:
    def __init__(self, m_water=500.0, initial_salt_pct=0.0, true_alpha=8.1, true_beta=0.15):
        # 🌟 水分量をデフォルトで500g（600g - 豆腐100g）に変更
        self.m_liquid_base = m_water 
        self.salt_mass = self.m_liquid_base * (initial_salt_pct / 100.0)
        self.added_salt = 0.0
        self.true_alpha = true_alpha
        self.true_beta = true_beta
        
    def add_salt(self, target_salt_g):
        # プロセスノイズ（アクチュエータのブレ：標準偏差5%）
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
        # 観測ノイズ（センサーの電気的ノイズ：標準偏差1%）
        noisy_ec = true_ec * np.random.normal(loc=1.0, scale=0.01)
        return noisy_ec

def run_simulation(true_alpha, true_beta, init_salt_pct=0.0, mode="STR"):
    # 🌟 豆腐100g投入により水分量が500gに減少した実機プラントを再現
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
# 4. ブロック型ヒートマップ（15cm・Miso+Tofuのみプロット版）
# ==========================================
if __name__ == "__main__":
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 9 
    
    # 🌟 変更点：豆腐100g入り味噌汁の初期塩分濃度（1.78g / 501.78g ≒ 0.3547%）
    INIT_SALT = 1.78 / (500.0 + 1.78) * 100  # ≒ 0.3547%
    N_TRIALS = 10
    
    RESOLUTION_X = 11  
    RESOLUTION_Y = 11  
    
    alpha_range = np.linspace(4.0, 14.0, RESOLUTION_X)
    beta_range = np.linspace(0.5,2.5, RESOLUTION_Y)
    
    A, B = np.meshgrid(alpha_range, beta_range)
    Z_pid = np.zeros_like(A)
    Z_str = np.zeros_like(A)
    
    print(f"確率論的シミュレーションを実行中... (初期塩分 {INIT_SALT:.4f}%, 各マス {N_TRIALS}回平均)")
    for i in range(RESOLUTION_Y):
        for j in range(RESOLUTION_X):
            pid_errors, str_errors = [], []
            for _ in range(N_TRIALS):
                pid_errors.append(run_simulation(A[i, j], B[i, j], init_salt_pct=INIT_SALT, mode="PID"))
                str_errors.append(run_simulation(A[i, j], B[i, j], init_salt_pct=INIT_SALT, mode="STR"))
            Z_pid[i, j] = np.mean(pid_errors)
            Z_str[i, j] = np.mean(str_errors)
            
    # 横幅15cm、縦幅7.5cm（正方形アスペクト比を維持するための比率）
    fig, axes = plt.subplots(1, 2, figsize=(15/2.54, 7.5/2.54), constrained_layout=True)
    
    VMIN, VMAX = -60, 60
    cmap = 'coolwarm'
    
    # 🌟 水分減少（500g）と初期塩分0.35%を同時に含む空間における「Miso+Tofu」の実機座標
    real_soups = [
        (8.5359, 1.1373, 'Miso+Tofu', 'lime')
    ]

    title_pid = f'(a) PID Control'
    title_str = f'(b) Adaptive Tasting Control'

    for ax, Z, title in zip(axes, [Z_pid, Z_str], [title_pid, title_str]):
        
        dx = (alpha_range[1] - alpha_range[0]) / 2.0
        dy = (beta_range[1] - beta_range[0]) / 2.0
        extent = [alpha_range[0]-dx, alpha_range[-1]+dx, beta_range[0]-dy, beta_range[-1]+dy]
        
        c = ax.imshow(Z, origin='lower', cmap=cmap, vmin=VMIN, vmax=VMAX, extent=extent, aspect='auto')
        
        # グラフ領域を完全な正方形にする
        ax.set_box_aspect(1)
        
        # マス目の数字描画
        for i in range(RESOLUTION_Y):
            for j in range(RESOLUTION_X):
                val = Z[i, j]
                text_color = 'white' if abs(val) > 15 else 'black'
                ax.text(alpha_range[j], beta_range[i], f'{val:.1f}', 
                        ha='center', va='center', color=text_color, fontsize=5.5, alpha=0.9)
        
        # 星マーク（今回は小さな丸マーク）とアノテーションの描画
        for a_val, b_val, name, color in real_soups:
            # マップを隠さない小さな黒縁丸マーク
            ax.plot(a_val, b_val, marker='o', markersize=6, color=color, markeredgecolor='black', zorder=5)
            
            # マスの数値（理論値）を極力隠さないよう、右上の安全な余白へ配置
            xytext = (30, 15)  
                
            ax.annotate(name, xy=(a_val, b_val), xytext=xytext,
                        textcoords='offset points', ha='center', va='center',
                        fontsize=8, weight='bold', color='black',
                        bbox=dict(boxstyle="round,pad=0.2", facecolor='white', alpha=0.9, edgecolor='gray'),
                        arrowprops=dict(arrowstyle="->", color="black", linewidth=1.0, connectionstyle="arc3,rad=0.1"), zorder=6)
            
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('True Parameter $\\alpha$', fontsize=9)
        ax.set_ylabel('True Parameter $\\beta$', fontsize=9)
        
        ax.set_xticks(alpha_range)
        ax.set_yticks(beta_range)
        ax.tick_params(axis='both', which='major', labelsize=8)
    
    cbar = fig.colorbar(c, ax=axes, shrink=0.8, aspect=20, pad=0.02)
    cbar.set_label('Final Concentration Error [%]', fontsize=9)
    cbar.ax.tick_params(labelsize=8)
    
   # ファイル名設定
    filename_png = f'Heatmap.png'
    filename_pdf = f'Heatmap.pdf'
    
    # 両方の形式で保存（PDFはベクター形式）
    plt.savefig(filename_png, dpi=300, bbox_inches='tight')
    plt.savefig(filename_pdf, bbox_inches='tight')
    
    print(f"完了！最適化した15cm幅画像を保存しました:")
    print(f"  - ラスター画像 (PNG): {filename_png}")
    print(f"  - ベクター画像 (PDF): {filename_pdf}")
    plt.show()