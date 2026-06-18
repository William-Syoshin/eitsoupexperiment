import numpy as np
import matplotlib.pyplot as plt
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController

# ==========================================
# 環境の固定設定（実機で最も過酷な Miso+Tofu を再現）
# ==========================================
TRUE_ALPHA = 8.5359  # Miso + Tofu
TRUE_BETA  = 1.1373  # Miso + Tofu
M_WATER    = 500.0   # 600g - 豆腐100g
M_SOLID    = 100.0

# 基準値（純水）
ALPHA_NOMINAL = 8.1013
BETA_NOMINAL  = 0.1550
TEMP_COEFF    = 0.02
T_BASE        = 24.6
T_CONST       = 50.0

# STR/PID設定
ALPHA_MIN, ALPHA_MAX = ALPHA_NOMINAL * 0.3, ALPHA_NOMINAL * 3.0
BETA_MIN,  BETA_MAX  = 0.0, 5.0
D_ALPHA_MAX, D_BETA_MAX = 0.5, 0.2
DEADBAND = 0.02
KP, KI, KD = 0.1, 0.00, 0.0
SALT_INTERVAL = 30.0
C_TARGET = 1.0
M_TOTAL_ASSUMED = 600.0

class FastSoupPlant:
    def __init__(self, m_water, true_alpha, true_beta):
        self.m_liquid_base = m_water 
        self.salt_mass = 0.0
        self.added_salt = 0.0
        self.true_alpha = true_alpha
        self.true_beta = true_beta
        
    def add_salt(self, salt_g):
        self.added_salt += salt_g
        self.salt_mass += salt_g
        
    @property
    def true_concentration(self):
        current_liquid_mass = self.m_liquid_base + self.added_salt
        if current_liquid_mass == 0: return 0.0
        return (self.salt_mass / current_liquid_mass) * 100.0
    
    @property
    def get_ec(self):
        C = self.true_concentration
        sigma_base = self.true_alpha * C + self.true_beta
        return sigma_base * (1.0 + TEMP_COEFF * (T_CONST - T_BASE))

# 💡 max_steps と salt_max を外部から弄れるように引数に追加！
def run_optimization_sim(max_steps, salt_max):
    plant = FastSoupPlant(m_water=M_WATER, true_alpha=TRUE_ALPHA, true_beta=TRUE_BETA)
    str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, lam=1.0)
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD, output_min=0.0, output_max=salt_max / SALT_INTERVAL)
    
    total_salt = 0.0
    alpha_h, beta_h = ALPHA_NOMINAL, BETA_NOMINAL
    prev_alpha, prev_beta = ALPHA_NOMINAL, BETA_NOMINAL
    C_init_virtual = 0.0
    zero_salt_count = 0
    
    for step in range(int(max_steps)):
        sigma_comp = plant.get_ec / (1.0 + TEMP_COEFF * (T_CONST - T_BASE))
        
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
        
        salt_g = min(salt_g, salt_max)
        
        if salt_g >= 0.01:
            plant.add_salt(salt_g)
            total_salt += salt_g
            zero_salt_count = 0
        else:
            zero_salt_count += 1
            
        if zero_salt_count >= 2:
            break

    return ((plant.true_concentration - C_TARGET) / C_TARGET) * 100.0

# ==========================================
# 最適化ヒートマップの描画
# ==========================================
if __name__ == "__main__":
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 11
    
    # 探索範囲の設定
    # 塩の上限：0.1g から 1.0g まで (10段階)
    salt_limits = np.linspace(0.3, 5.0, 10)
    # ステップ数：10回 から 28回 まで (10段階)
    max_steps_range = np.linspace(3, 30, 10)
    
    A, B = np.meshgrid(salt_limits, max_steps_range)
    Z = np.zeros_like(A)
    
    print("最適化シミュレーションを実行中...")
    for i in range(len(max_steps_range)):
        for j in range(len(salt_limits)):
            Z[i, j] = run_optimization_sim(max_steps=B[i, j], salt_max=A[i, j])
            
    fig, ax = plt.subplots(figsize=(10, 8))
    
    cmap = 'coolwarm'
    VMIN, VMAX = -30, 30
    
    dx = (salt_limits[1] - salt_limits[0]) / 2.0
    dy = (max_steps_range[1] - max_steps_range[0]) / 2.0
    extent = [salt_limits[0]-dx, salt_limits[-1]+dx, max_steps_range[0]-dy, max_steps_range[-1]+dy]
    
    c = ax.imshow(Z, origin='lower', cmap=cmap, vmin=VMIN, vmax=VMAX, extent=extent, aspect='auto')
    
    # 各マスにエラー率を記入
    for i in range(len(max_steps_range)):
        for j in range(len(salt_limits)):
            val = Z[i, j]
            text_color = 'white' if abs(val) > 15 else 'black'
            ax.text(salt_limits[j], max_steps_range[i], f'{val:.1f}', 
                    ha='center', va='center', color=text_color, fontsize=9, fontweight='bold')
    
    ax.set_title('Process Optimization for Miso+Tofu Environment (STR)', fontsize=14, pad=15)
    ax.set_xlabel('Actuator Limit (Max Salt per Step) [g]', fontsize=12)
    ax.set_ylabel('Total Allowed Steps', fontsize=12)
    
    ax.set_xticks(salt_limits)
    ax.set_yticks(max_steps_range)
    
    cbar = fig.colorbar(c, ax=ax, fraction=0.04, pad=0.04)
    cbar.set_label('Final Concentration Error [%]', fontsize=12)
    
    plt.tight_layout()
    plt.savefig('Sim_Optimization_Heatmap.png', dpi=300)
    print("完了！最適化ヒートマップを保存しました。")
    plt.show()