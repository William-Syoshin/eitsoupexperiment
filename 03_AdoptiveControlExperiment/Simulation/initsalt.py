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

class FastSoupPlant:
    def __init__(self, m_water=500.0, initial_salt_pct=0.0):
        # 🌟 実機 Miso+Tofu の過酷な物理環境に完全固定！
        self.m_liquid_base = m_water 
        self.salt_mass = self.m_liquid_base * (initial_salt_pct / 100.0)
        self.added_salt = 0.0
        self.true_alpha = 8.5359  # Miso+Tofu の実際の感度
        self.true_beta = 1.1373   # Miso+Tofu の実際のベースライン
        
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

def run_simulation(init_salt_pct, mode="STR"):
    plant = FastSoupPlant(m_water=500.0, initial_salt_pct=init_salt_pct)
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
# 描画処理：エラーバンド（帯）付き折れ線グラフ
# ==========================================
if __name__ == "__main__":
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 10
    
    # 🌟 初期塩分濃度を 0.0% から 0.9% まで振る
    salt_range = np.linspace(0.0, 0.9, 19)
    N_TRIALS = 30  # モンテカルロ試行回数（帯を描くために少し多めに）
    
    pid_means, pid_stds = [], []
    str_means, str_stds = [], []
    
    print("初期塩分濃度の不確実性シミュレーションを実行中...")
    for init_salt in salt_range:
        err_pid = []
        err_str = []
        for _ in range(N_TRIALS):
            err_pid.append(run_simulation(init_salt, mode="PID"))
            err_str.append(run_simulation(init_salt, mode="STR"))
            
        pid_means.append(np.mean(err_pid))
        pid_stds.append(np.std(err_pid))
        str_means.append(np.mean(err_str))
        str_stds.append(np.std(err_str))
        
    pid_means = np.array(pid_means)
    pid_stds = np.array(pid_stds)
    str_means = np.array(str_means)
    str_stds = np.array(str_stds)

    # 🌟 論文の片側カラム（約10cm幅）に美しく収まるサイズ設定
    fig, ax = plt.subplots(figsize=(10.5/2.54, 7.5/2.54), constrained_layout=True)
    
    # 目標の完璧なライン（エラー 0%）
    ax.axhline(0, color='black', linestyle='--', linewidth=1.2, alpha=0.8, label='Target (0% Error)')
    
    # PID の描画（青色）
    ax.plot(salt_range, pid_means, color='#1f77b4', label='Fixed PID', linewidth=2.0)
    ax.fill_between(salt_range, pid_means - pid_stds, pid_means + pid_stds, color='#1f77b4', alpha=0.2)
    
    # STR の描画（緑色）
    ax.plot(salt_range, str_means, color='#2ca02c', label='Adaptive Tasting (STR)', linewidth=2.0)
    ax.fill_between(salt_range, str_means - str_stds, str_means + str_stds, color='#2ca02c', alpha=0.3)
    
    # 実機の初期塩分座標をエモく示すアノテーション
    actual_salt = 0.35
    ax.axvline(actual_salt, color='gray', linestyle=':', linewidth=1.5)
    ax.annotate('Actual Miso+Tofu\nInitial Salt (0.35%)', xy=(actual_salt, 10), xytext=(actual_salt + 0.05, 20),
                ha='left', va='center', fontsize=8, weight='bold', color='#444444',
                bbox=dict(boxstyle="round,pad=0.2", facecolor='white', alpha=0.9, edgecolor='gray'),
                arrowprops=dict(arrowstyle="->", color="gray", linewidth=1.2, connectionstyle="arc3,rad=-0.2"))

    ax.set_title('Robustness Against Unknown Initial Salt', fontsize=11, fontweight='bold', pad=8)
    ax.set_xlabel('Initial Salt Concentration [%]', fontsize=10)
    ax.set_ylabel('Final Concentration Error [%]', fontsize=10)
    
    ax.set_xlim(0.0, 0.9)
    ax.set_ylim(-35, 35)
    
    # グリッドと凡例
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.legend(fontsize=8, loc='lower right', framealpha=0.9)
    
    filename_png = 'Initial_Salt_Robustness.png'
    filename_pdf = 'Initial_Salt_Robustness.pdf'
    plt.savefig(filename_png, dpi=300, bbox_inches='tight')
    plt.savefig(filename_pdf, bbox_inches='tight')
    
    print(f"完了！初期塩分ロバスト性グラフを保存しました:\n  - {filename_png}\n  - {filename_pdf}")
    plt.show()