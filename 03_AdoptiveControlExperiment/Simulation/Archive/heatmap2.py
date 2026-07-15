import numpy as np
import matplotlib.pyplot as plt
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController

# ==========================================
# 共通定数・プラント・シミュレーション関数
# (※前回のコードの 1.〜3. と全く同じものを使用します)
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
    def __init__(self, m_water=600.0, initial_salt_pct=0.0, true_alpha=8.1, true_beta=0.15):
        self.m_liquid_base = m_water 
        self.salt_mass = self.m_liquid_base * (initial_salt_pct / 100.0)
        self.added_salt = 0.0
        self.true_alpha = true_alpha
        self.true_beta = true_beta
        
    def add_salt(self, salt_g):
        self.added_salt += salt_g
        self.salt_mass += salt_g
        
    @property
    def true_concentration(self):
        return (self.salt_mass / (self.m_liquid_base + self.added_salt)) * 100.0
    
    @property
    def get_ec(self):
        sigma_base = self.true_alpha * self.true_concentration + self.true_beta
        return sigma_base * (1.0 + TEMP_COEFF * (T_CONST - T_BASE))

# 💡 init_salt_pct を追加し、FastSoupPlant に渡す
def run_simulation(true_alpha, true_beta, init_salt_pct=0.0, mode="STR"):
    plant = FastSoupPlant(m_water=600.0, initial_salt_pct=init_salt_pct, 
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
# 4. ブロック型ヒートマップ（Annotated Heatmap）の描画
# ==========================================
if __name__ == "__main__":
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 10
    
    # 💡 1. 任意の初期塩分濃度をここで設定！
    INIT_SALT = 0.2
    
    RESOLUTION_X = 11  
    RESOLUTION_Y = 11  
    
    alpha_range = np.linspace(5.0, 15.0, RESOLUTION_X)
    beta_range = np.linspace(0.0, 2.0, RESOLUTION_Y)
    
    A, B = np.meshgrid(alpha_range, beta_range)
    Z_pid = np.zeros_like(A)
    Z_str = np.zeros_like(A)
    
    print(f"シミュレーションを実行中... (初期塩分 {INIT_SALT}%)")
    for i in range(RESOLUTION_Y):
        for j in range(RESOLUTION_X):
            # 💡 2. ここで init_salt_pct を渡して計算
            Z_pid[i, j] = run_simulation(A[i, j], B[i, j], init_salt_pct=INIT_SALT, mode="PID")
            Z_str[i, j] = run_simulation(A[i, j], B[i, j], init_salt_pct=INIT_SALT, mode="STR")
            
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    VMIN, VMAX = -50, 50
    cmap = 'coolwarm'
    
    real_soups = [
        (8.1013, 0.1550, 'Water', 'white'),
        (9.1819, 1.1606, 'Miso', 'yellow'),
        (8.5359, 1.1373, 'Miso+Tofu', 'lime')
    ]

    # 💡 3. タイトル文字列に INIT_SALT を自動で埋め込む
    title_pid = f'(a) PID Control Robustness (Init Salt: {INIT_SALT}%)'
    title_str = f'(b) STR Control Robustness (Init Salt: {INIT_SALT}%)'

    for ax, Z, title in zip(axes, [Z_pid, Z_str], [title_pid, title_str]):
        
        dx = (alpha_range[1] - alpha_range[0]) / 2.0
        dy = (beta_range[1] - beta_range[0]) / 2.0
        extent = [alpha_range[0]-dx, alpha_range[-1]+dx, beta_range[0]-dy, beta_range[-1]+dy]
        
        c = ax.imshow(Z, origin='lower', cmap=cmap, vmin=VMIN, vmax=VMAX, extent=extent, aspect='auto')
        
        for i in range(RESOLUTION_Y):
            for j in range(RESOLUTION_X):
                val = Z[i, j]
                text_color = 'white' if abs(val) > 15 else 'black'
                ax.text(alpha_range[j], beta_range[i], f'{val:.1f}', 
                        ha='center', va='center', color=text_color, fontsize=8)
        
        for a_val, b_val, name, color in real_soups:
            ax.plot(a_val, b_val, marker='*', markersize=14, color=color, markeredgecolor='black', zorder=5)
            ax.text(a_val, b_val + 0.08, name, fontsize=9, color='black', weight='bold', ha='center',
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1), zorder=6)
            
        # 💡 自動生成したタイトルをセット
        ax.set_title(title)
        ax.set_xlabel('True Parameter $\\alpha$')
        ax.set_ylabel('True Parameter $\\beta$')
        
        ax.set_xticks(alpha_range)
        ax.set_yticks(beta_range)
    
    cbar = fig.colorbar(c, ax=axes.ravel().tolist(), fraction=0.02, pad=0.04)
    cbar.set_label('Final Concentration Error [%]')
    
    # 💡 4. 保存するファイル名にも塩分濃度を入れておくと後で分かりやすいです
    filename = f'Sim_BlockHeatmap_Salt_{INIT_SALT}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"完了！画像を保存しました: {filename}")
    plt.show()