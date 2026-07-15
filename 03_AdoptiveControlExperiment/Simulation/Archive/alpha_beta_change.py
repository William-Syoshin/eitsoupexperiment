import numpy as np
import matplotlib.pyplot as plt
from Controls.PIDcontrol import PIDController
from Controls.OpenLoop import OpenLoopController
from Controls.AdaptiveControl import STRController

# ==========================================
# 1. 共通定数の設定
# ==========================================
MAX_STEPS = 24
SALT_INTERVAL = 30.0
C_TARGET = 1.0
SALT_MAX_PER_STEP = 0.5
M_TOTAL_ASSUMED = 600.0  # ロボットが思い込んでいる総質量

# ロボットの初期知識（キャリブレーション値）
ALPHA_NOMINAL = 8.1013
BETA_NOMINAL  = 0.1550
TEMP_COEFF    = 0.02
T_BASE        = 24.6

# STRの安全制限
ALPHA_MIN = ALPHA_NOMINAL / 3.0
ALPHA_MAX = ALPHA_NOMINAL * 3.0
BETA_MIN  = -5.0
BETA_MAX  = 5.0
D_ALPHA_MAX = 0.5
D_BETA_MAX  = 0.2
DEADBAND    = 0.01

# PIDの基本ゲイン
KP = 0.1
KI = 0.0#0.005
KD = 0.0

# 温度
T_CONST = 50.0 

# ==========================================
# 2. プラントシミュレータ
# ==========================================
class FastSoupPlant:
    def __init__(self, m_water=600.0, m_solid=0.0, initial_salt_pct=0.0, true_alpha=8.1, true_beta=0.15):
        self.m_water = m_water
        self.m_solid = m_solid
        self.m_liquid_base = self.m_water 
        self.salt_mass = self.m_liquid_base * (initial_salt_pct / 100.0)
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

# ==========================================
# 3. コア関数（コントロールモードを指定可能に）
# ==========================================
def run_simulation(true_alpha, true_beta, m_water=600.0, m_solid=0.0, init_salt_pct=0.0, mode="STR"):
    plant = FastSoupPlant(m_water=m_water, m_solid=m_solid, 
                          initial_salt_pct=init_salt_pct, 
                          true_alpha=true_alpha, true_beta=true_beta)
    
    str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, lam=1.0)
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD, output_min=0.0, output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)
    
    total_salt = 0.0
    alpha_h = ALPHA_NOMINAL
    beta_h  = BETA_NOMINAL
    prev_alpha = ALPHA_NOMINAL
    prev_beta  = BETA_NOMINAL
    C_init_virtual = 0.0
    
    # 💡 新しい状態変数：連続0gカウント
    zero_salt_count = 0
    
    for step in range(MAX_STEPS):
        sigma = plant.get_ec
        salt_g = 0.0
        
        temp_factor = 1.0 + TEMP_COEFF * (T_CONST - T_BASE)
        sigma_comp = sigma / temp_factor
        
        if mode == "OpenLoop":
            if step == 0:
                salt_g = (C_TARGET / 100.0) * M_TOTAL_ASSUMED
            else:
                salt_g = 0.0
                
        elif mode == "PID":
            C_hat = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
            error = C_TARGET - C_hat
            rate = pid.compute(error, SALT_INTERVAL)
            salt_g = rate * SALT_INTERVAL
            
        elif mode == "STR":
            if total_salt < 0.01:
                C_init_virtual = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                prev_beta = BETA_NOMINAL
                
            C_added = (total_salt / M_TOTAL_ASSUMED) * 100.0
            X_virtual_total = C_init_virtual + C_added
            
            raw_alpha_h, raw_beta_h = str_unit.estimate(sigma_comp, X_virtual_total)
            
            d_alpha = raw_alpha_h - prev_alpha
            d_beta  = raw_beta_h - prev_beta
            if abs(d_alpha) < DEADBAND: d_alpha = 0.0
            if abs(d_beta)  < DEADBAND: d_beta  = 0.0
            d_alpha = max(-D_ALPHA_MAX, min(d_alpha, D_ALPHA_MAX))
            d_beta  = max(-D_BETA_MAX,  min(d_beta,  D_BETA_MAX))
            
            alpha_h = max(ALPHA_MIN, min(prev_alpha + d_alpha, ALPHA_MAX))
            beta_h  = max(BETA_MIN,  min(prev_beta  + d_beta,  BETA_MAX))
            
            prev_alpha, prev_beta = alpha_h, beta_h
            str_unit.theta[0, 0] = alpha_h
            str_unit.theta[1, 0] = beta_h
            
            adaptive_ratio = min(max(ALPHA_NOMINAL / alpha_h, 0.3), 3.0)
            pid.Kp = KP * adaptive_ratio
            pid.Ki = KI * adaptive_ratio
            pid.Kd = KD * adaptive_ratio
            
            if total_salt < 0.01:
                C_true_abs = C_init_virtual
            else:
                C_true_abs = (sigma_comp - beta_h) / alpha_h
                
            error = C_TARGET - C_true_abs
            salt_g = pid.compute(error, SALT_INTERVAL) * SALT_INTERVAL
            
        # 投入量制限
        salt_g = min(salt_g, SALT_MAX_PER_STEP) if mode != "OpenLoop" else salt_g
        
        # 💡 実験ルールの適用（0.01g未満を切り捨て、連続カウントをチェック）
        if salt_g >= 0.01:
            plant.add_salt(salt_g)
            total_salt += salt_g
            zero_salt_count = 0  # 塩を入れたらカウントリセット
        else:
            zero_salt_count += 1 # 塩を入れなかったらカウントアップ
            
        # 💡 2回連続で塩投入がなければ、実験を早期終了する
        if zero_salt_count >= 2:
            break

    # エラー率の計算（breakしてループを抜けた瞬間の濃度が最終評価になる）
    final_C = plant.true_concentration
    error_rate = ((final_C - C_TARGET) / C_TARGET) * 100.0
    
    return error_rate

# ==========================================
# 4. グラフ描画（初期塩分濃度を引数で指定可能に拡張）
# ==========================================
def plot_alpha_sweep_all(ax, init_salt=0.0):
    alphas = np.linspace(4.0, 16.0, 30)
    err_str, err_pid, err_open = [], [], []
    
    for a in alphas:
        # 💡 ここで init_salt を run_simulation に渡す
        err_str.append(run_simulation(true_alpha=a, true_beta=BETA_NOMINAL, init_salt_pct=init_salt, mode="STR"))
        err_pid.append(run_simulation(true_alpha=a, true_beta=BETA_NOMINAL, init_salt_pct=init_salt, mode="PID"))
        err_open.append(run_simulation(true_alpha=a, true_beta=BETA_NOMINAL, init_salt_pct=init_salt, mode="OpenLoop"))
        
    ax.plot(alphas, err_open, color='gray', lw=1.5, label='Open-Loop', zorder=1)
    ax.plot(alphas, err_pid, color='#1f77b4', lw=2, marker='^', ms=4, label='PID', zorder=2)
    ax.plot(alphas, err_str, color='#d62728', lw=2.5, marker='o', ms=5, label='STR', zorder=3)
    
    ax.axhline(0, color='black', ls='--', lw=1)
    ax.axvline(ALPHA_NOMINAL, color='green', ls=':', label='Nominal $\\alpha$')
    
    ax.set_ylim(-40, 40)
    
    # 💡 タイトルに初期塩分濃度を動的に追加
    ax.set_title(f'Robustness against True $\\alpha$ (Init Salt: {init_salt}%)')
    ax.set_xlabel('True Parameter $\\alpha$')
    ax.set_ylabel('Final Concentration Error [%]')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

def plot_beta_sweep_all(ax, init_salt=0.0):
    betas = np.linspace(0.0, 2.0, 30)
    err_str, err_pid, err_open = [], [], []
    
    for b in betas:
        # 💡 ここで init_salt を run_simulation に渡す
        err_str.append(run_simulation(true_alpha=ALPHA_NOMINAL, true_beta=b, init_salt_pct=init_salt, mode="STR"))
        err_pid.append(run_simulation(true_alpha=ALPHA_NOMINAL, true_beta=b, init_salt_pct=init_salt, mode="PID"))
        err_open.append(run_simulation(true_alpha=ALPHA_NOMINAL, true_beta=b, init_salt_pct=init_salt, mode="OpenLoop"))
        
    ax.plot(betas, err_open, color='gray', lw=1.5, label='Open-Loop', zorder=1)
    ax.plot(betas, err_pid, color='#1f77b4', lw=2, marker='^', ms=4, label='PID', zorder=2)
    ax.plot(betas, err_str, color='#d62728', lw=2.5, marker='o', ms=5, label='STR', zorder=3)
    
    ax.axhline(0, color='black', ls='--', lw=1)
    ax.axvline(BETA_NOMINAL, color='green', ls=':', label='Nominal $\\beta$')
    
    ax.set_ylim(-40, 40)
    
    # 💡 タイトルに初期塩分濃度を動的に追加
    ax.set_title(f'Robustness against True $\\beta$ (Init Salt: {init_salt}%)')
    ax.set_xlabel('True Parameter $\\beta$')
    ax.set_ylabel('Final Concentration Error [%]')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

# ==========================================
# 5. メイン実行部（2行2列のレイアウト）
# ==========================================
if __name__ == "__main__":
    # 💡 フォントとサイズの指定（Times New Roman, サイズ10）
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 10
    
    # 💡 2行2列のグラフ（縦を少し長めの 8.0 インチに設定）
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.0))
    
    # 上段 (Row 0): 初期塩分 0.0%
    plot_alpha_sweep_all(axes[0, 0], init_salt=0.0)
    plot_beta_sweep_all(axes[0, 1], init_salt=0.0)
    
    # 下段 (Row 1): 初期塩分 0.2%
    plot_alpha_sweep_all(axes[1, 0], init_salt=0.2)
    plot_beta_sweep_all(axes[1, 1], init_salt=0.2)
    
    plt.tight_layout()
    plt.savefig('Sim_Robustness_VaryingSalt.png', dpi=300)
    print("2パターンのシミュレーション完了！グラフを保存しました。")
    plt.show()