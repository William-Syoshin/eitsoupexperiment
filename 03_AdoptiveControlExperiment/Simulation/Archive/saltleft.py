# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from Controls.PIDcontrol import PIDController
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
D_ALPHA_MAX, D_BETA_MAX = 0.5, 0.2
DEADBAND = 0.02
KP, KI, KD = 0.1, 0.00, 0.0
T_CONST = 50.0

class FastSoupPlantWithMixing:
    def __init__(self, m_water=500.0, initial_salt_pct=0.35, mix_efficiency=1.0):
        self.m_liquid_base = m_water 
        # 最初から入っている下味はすでに完全に溶け切っているとする
        self.salt_mass_dissolved = self.m_liquid_base * (initial_salt_pct / 100.0)
        # ロボットが後から入れた「まだ溶けていない塩」
        self.salt_mass_undissolved = 0.0 
        
        self.mix_efficiency = mix_efficiency  # 1ステップあたりに溶ける割合 (0.0 ~ 1.0)
        self.true_alpha = 8.5359  # Miso+Tofu
        self.true_beta = 1.1373
        
    def add_salt(self, target_salt_g):
        actual_salt_g = target_salt_g * np.random.normal(loc=1.0, scale=0.05)
        # 追加された塩は、まずは「溶け残り」として蓄積される
        self.salt_mass_undissolved += actual_salt_g

    def step_mixing(self):
        # 🌟 時間経過（混ぜる行為）による溶解ダイナミクス
        # 溶け残っている塩のうち、mix_efficiency の割合だけが新たに溶け込む
        dissolved_amount = self.salt_mass_undissolved * self.mix_efficiency
        self.salt_mass_undissolved -= dissolved_amount
        self.salt_mass_dissolved += dissolved_amount
        
    @property
    def current_observable_concentration(self):
        # 🌟 センサーが感知できるのは「すでに溶けている塩」だけ！
        return (self.salt_mass_dissolved / (self.m_liquid_base + self.salt_mass_dissolved)) * 100.0
    
    @property
    def final_true_concentration(self):
        # 🌟 食べる直前（最終結果）は、時間が経って「溶け残り」も含めて全て溶け切った状態
        total_salt = self.salt_mass_dissolved + self.salt_mass_undissolved
        return (total_salt / (self.m_liquid_base + total_salt)) * 100.0

    @property
    def get_ec(self):
        sigma_base = self.true_alpha * self.current_observable_concentration + self.true_beta
        true_ec = sigma_base * (1.0 + TEMP_COEFF * (T_CONST - T_BASE))
        noisy_ec = true_ec * np.random.normal(loc=1.0, scale=0.01)
        return noisy_ec

def run_simulation(mix_eff, mode="STR"):
    # Miso+Tofu の実機環境（初期塩分 0.35%）で固定し、混ざりやすさだけを変える
    plant = FastSoupPlantWithMixing(m_water=500.0, initial_salt_pct=0.35, mix_efficiency=mix_eff)
    str_unit = STRController(alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL, lam=1.0)
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD, output_min=0.0, output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)
    
    total_added_salt, zero_salt_count = 0.0, 0
    alpha_h, beta_h = ALPHA_NOMINAL, BETA_NOMINAL
    prev_alpha, prev_beta = ALPHA_NOMINAL, BETA_NOMINAL
    C_init_virtual = 0.0
    
    for step in range(MAX_STEPS):
        # 1. センサーで測定（まだ溶け切っていない薄い数値を観測してしまうかもしれない）
        sigma_comp = plant.get_ec / (1.0 + TEMP_COEFF * (T_CONST - T_BASE))
        salt_g = 0.0
        
        # 2. コントローラの計算
        if mode == "PID":
            C_hat = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
            salt_g = pid.compute(C_TARGET - C_hat, SALT_INTERVAL) * SALT_INTERVAL
            
        elif mode == "STR":
            if total_added_salt < 0.01:
                C_init_virtual = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                prev_beta = BETA_NOMINAL
            
            X_virtual_total = C_init_virtual + (total_added_salt / M_TOTAL_ASSUMED) * 100.0
            raw_alpha_h, raw_beta_h = str_unit.estimate(sigma_comp, X_virtual_total)
            
            d_alpha = max(-D_ALPHA_MAX, min(raw_alpha_h - prev_alpha if abs(raw_alpha_h - prev_alpha) >= DEADBAND else 0.0, D_ALPHA_MAX))
            d_beta  = max(-D_BETA_MAX,  min(raw_beta_h - prev_beta  if abs(raw_beta_h - prev_beta)  >= DEADBAND else 0.0, D_BETA_MAX))
            
            alpha_h = max(ALPHA_MIN, min(prev_alpha + d_alpha, ALPHA_MAX))
            beta_h  = max(BETA_MIN,  min(prev_beta  + d_beta,  BETA_MAX))
            prev_alpha, prev_beta = alpha_h, beta_h
            str_unit.theta[0, 0], str_unit.theta[1, 0] = alpha_h, beta_h
            
            adapt_ratio = min(max(ALPHA_NOMINAL / alpha_h, 0.3), 3.0)
            pid.Kp, pid.Ki, pid.Kd = KP * adapt_ratio, KI * adapt_ratio, KD * adapt_ratio
            
            C_true_abs = C_init_virtual if total_added_salt < 0.01 else (sigma_comp - beta_h) / alpha_h
            salt_g = pid.compute(C_TARGET - C_true_abs, SALT_INTERVAL) * SALT_INTERVAL
            
        salt_g = min(salt_g, SALT_MAX_PER_STEP)
        
        # 3. 塩の投入
        if salt_g >= 0.01:
            plant.add_salt(salt_g)
            total_added_salt += salt_g
            zero_salt_count = 0
        else:
            zero_salt_count += 1
            if zero_salt_count >= 2: break
            
        # 4. 次のステップに向けて「かき混ぜる（時間が経過して少し溶ける）」
        plant.step_mixing()

    # 最終結果は、食べる直前に「溶け残り」が全て溶け切った本当の濃度で評価する
    return ((plant.final_true_concentration - C_TARGET) / C_TARGET) * 100.0

# ==========================================
# 描画処理：エラーバンド（帯）付き折れ線グラフ
# ==========================================
if __name__ == "__main__":
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 10
    
    # 🌟 混合効率を 0.2（ドロドロで溶けない）〜 1.0（一瞬で溶ける水）まで振る
    mix_range = np.linspace(0.2, 1.0, 17)
    N_TRIALS = 30 
    
    pid_means, pid_stds = [], []
    str_means, str_stds = [], []
    
    print("不完全な撹拌（Incomplete Mixing）のロバスト性シミュレーションを実行中...")
    for mix_eff in mix_range:
        err_pid = []
        err_str = []
        for _ in range(N_TRIALS):
            err_pid.append(run_simulation(mix_eff, mode="PID"))
            err_str.append(run_simulation(mix_eff, mode="STR"))
            
        pid_means.append(np.mean(err_pid))
        pid_stds.append(np.std(err_pid))
        str_means.append(np.mean(err_str))
        str_stds.append(np.std(err_str))
        
    pid_means = np.array(pid_means)
    pid_stds = np.array(pid_stds)
    str_means = np.array(str_means)
    str_stds = np.array(str_stds)

    fig, ax = plt.subplots(figsize=(10.5/2.54, 7.5/2.54), constrained_layout=True)
    
    ax.axhline(0, color='black', linestyle='--', linewidth=1.2, alpha=0.8, label='Target (0% Error)')
    
    # PID の描画（青色）
    ax.plot(mix_range, pid_means, color='#1f77b4', label='Fixed PID', linewidth=2.0)
    ax.fill_between(mix_range, pid_means - pid_stds, pid_means + pid_stds, color='#1f77b4', alpha=0.2)
    
    # STR の描画（緑色）
    ax.plot(mix_range, str_means, color='#2ca02c', label='Adaptive Tasting (STR)', linewidth=2.0)
    ax.fill_between(mix_range, str_means - str_stds, str_means + str_stds, color='#2ca02c', alpha=0.3)
    
    # アノテーション
    ax.annotate('Poor Mixing\n(Tofu, Viscous)', xy=(0.3, 20), ha='center', va='center', fontsize=9, color='brown', weight='bold')
    ax.annotate('Instant Mixing\n(Pure Water)', xy=(0.9, -25), ha='center', va='center', fontsize=9, color='blue', weight='bold')

    ax.set_title('Robustness Against Incomplete Mixing Dynamics', fontsize=11, fontweight='bold', pad=8)
    # x軸は右に行くほど「溶けやすい（効率1.0）」、左に行くほど「溶けにくい（効率0.2）」
    ax.set_xlabel('Mixing Efficiency $k$ (Fraction Dissolved per Step)', fontsize=10)
    ax.set_ylabel('Final Concentration Error [%]', fontsize=10)
    
    ax.set_xlim(0.2, 1.0)
    ax.set_ylim(-30, 60) # PIDがオーバーシュートして上に吹っ飛ぶのでY軸を上に広げる
    
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    
    filename_png = 'Mixing_Dynamics_Robustness.png'
    filename_pdf = 'Mixing_Dynamics_Robustness.pdf'
    plt.savefig(filename_png, dpi=300, bbox_inches='tight')
    plt.savefig(filename_pdf, bbox_inches='tight')
    
    print(f"完了！撹拌ダイナミクスのロバスト性グラフを保存しました:\n  - {filename_png}\n  - {filename_pdf}")
    plt.show()