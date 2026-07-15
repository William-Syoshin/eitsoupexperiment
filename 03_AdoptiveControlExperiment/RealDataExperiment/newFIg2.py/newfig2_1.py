# -*- coding: utf-8 -*-
import io
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. 生データの入力エリア
# ==========================================
data_env1 = """
試行    C (g)   T (°C)  σ (mS/cm)
1   0   50.73   0.242
1   1   53.05   2.160
1   2   52.26   4.610
1   3   51.31   6.210
1   4   50.66   7.940
1   5   49.98   10.12
1   6   51.21   12.00
1   7   53.45   13.68
1   8   53.22   15.14
3   0   49.33   0.218
3   1   50.39   2.820
3   2   50.32   5.350
3   3   49.74   7.720
3   4   51.62   9.220
3   5   51.62   9.720
3   6   50.35   12.61
3   7   49.35   13.77
3   8   48.55   16.55
4   0   51.72   0.247
4   1   49.81   2.770
4   2   48.75   4.870
4   3   50.01   7.210
4   4   48.65   9.180
4   5   50.76   11.21
4   6   50.23   12.99
4   7   50.35   14.96
4   8   50.05   16.40
"""

data_env2 = """
試行    C (g)   T (°C)  σ (mS/cm)
1   0   51.99   6.16
1   1   51.62   8.38
1   2   50.35   10.8
1   3   49.49   13.33
1   4   50.56   16.35
1   5   51.27   19.15
1   6   50.42   21.7
1   7   49.43   22.3
1   8   52.00   24.5
2   0   53.70   5.53
2   1   51.92   6.93
2   2   50.42   8.75
2   3   51.51   11.71
2   4   51.85   13.99
2   5   48.45   17.4
2   6   46.74   19.01
2   7   49.16   21.0
2   8   52.47   21.9
3   0   51.21   6.13
3   1   50.12   8.38
3   2   48.56   10.66
3   3   51.85   13.09
3   4   50.76   15.79
3   5   49.26   18.23
3   6   51.82   20.0
3   7   49.91   20.9
3   8   48.50   22.6
"""

data_env3 = """
試行    C (g)   T (°C)  σ (mS/cm)
1   0   54.28   6.42
1   1   54.5    8.01
1   2   54.14   11.02
1   3   52.82   15.35
1   4   51.55   18.41
1   5   50.76   21.6
1   6   49.77   23.3
1   7   54.17   24.9
1   8   52.84   26.8
2   0   54.07   5.8
2   1   52.7    8.08
2   2   51.7    11.26
2   3   49.44   14.66
2   4   51.65   17.86
2   5   50.08   20.8
2   6   51.72   22.3
2   7   51  23.4
2   8   49.57   24
3   0   53.76   6.88
3   1   52.47   8.15
3   2   50.42   10.04
3   3   51.14   12.44
3   4   51.99   15.76
3   5   52.37   18.95
3   6   51.51   21.3
3   7   48.51   22.4
3   8   50.32   25.4
"""

# ==========================================
# 2. データ抽出と「温度補正」の計算
# ==========================================
T_room = 24.6
a_temp = 0.02

def extract_raw_data(data_text, m_base_liquid, initial_salt):
    df = pd.read_csv(io.StringIO(data_text.strip()), sep=r'\s+', skiprows=1, names=['Trial', 'C_g', 'T_C', 'Sigma'])
    df = df.dropna()
    # 濃度計算
    df['C_pct'] = ((df['C_g'] + initial_salt) / (m_base_liquid + df['C_g'])) * 100
    # 🌟 ここで「温度補正」をかけた仮想の伝導率を計算（PIDが見ている世界）
    df['Sigma_comp'] = df['Sigma'] / (1 + a_temp * (df['T_C'] - T_room))
    return df

miso_initial_salt = 20.0 * 0.087

df_w = extract_raw_data(data_env1, 600.0, 0.0)               
df_m = extract_raw_data(data_env2, 600.0, miso_initial_salt) 
df_t = extract_raw_data(data_env3, 500.0, miso_initial_salt) 

# ==========================================
# 3. グラフ描画（1行2列）
# ==========================================
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.size'] = 9

fig, axes = plt.subplots(1, 2, figsize=(15 / 2.54, 7.0 / 2.54), constrained_layout=True)

def plot_trajectories(ax, df, x_col, y_col, color, marker, label):
    # 直線トレンドとばらつき（標準偏差）の帯を計算
    z = np.polyfit(df[x_col], df[y_col], 1)
    p = np.poly1d(z)
    x_range = np.linspace(df[x_col].min(), df[x_col].max(), 100)
    
    res = df[y_col] - p(df[x_col])
    std_res = np.std(res)
    
    ax.fill_between(x_range, p(x_range) - 2*std_res, p(x_range) + 2*std_res, color=color, alpha=0.15, zorder=1)
    
    # 試行ごとのウネウネ軌跡を描画
    trials = df['Trial'].unique()
    for i, trial_num in enumerate(trials):
        trial_df = df[df['Trial'] == trial_num]
        lbl = label if i == 0 else ""
        ax.plot(trial_df[x_col], trial_df[y_col], color=color, marker=marker, markersize=4, 
                linestyle='-', linewidth=1.5, alpha=0.8, label=lbl, zorder=3)

# ----------------------------------------------------
# 左図 (a): Raw Conductivity (温度補正なしの完全なカオス)
# ----------------------------------------------------
ax1 = axes[0]
plot_trajectories(ax1, df_w, 'C_pct', 'Sigma', '#1f77b4', 'o', 'Pure Water')
plot_trajectories(ax1, df_m, 'C_pct', 'Sigma', '#ff7f0e', 's', 'Miso Soup')
plot_trajectories(ax1, df_t, 'C_pct', 'Sigma', '#ce0000', 'D', 'Miso+Tofu')

# ユーザー様が見せたかった「温度低下による逆走・カオス」を強調
ax1.annotate('Temperature Drop\n(Sensors go chaotic!)', xy=(1.5, 23.5), xytext=(0.8, 25),
             ha='center', va='center', fontsize=8, fontweight='bold', color='#ce0000',
             arrowprops=dict(arrowstyle="->", color="#ce0000", linewidth=1.2, connectionstyle="arc3,rad=-0.2"))

ax1.set_title('(a) Raw Conductivity $\\sigma_{raw}$', fontsize=10, fontweight='bold')
ax1.set_xlabel('True Salinity Concentration $C$ [%]')
ax1.set_ylabel('Raw EC [mS/cm]')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.set_xlim(-0.05, 2.0)
ax1.set_ylim(-2, 30)

# ----------------------------------------------------
# 右図 (b): Compensated Conductivity (温度補正後・PIDの絶望)
# ----------------------------------------------------
ax2 = axes[1]
plot_trajectories(ax2, df_w, 'C_pct', 'Sigma_comp', '#1f77b4', 'o', 'Pure Water')
plot_trajectories(ax2, df_m, 'C_pct', 'Sigma_comp', '#ff7f0e', 's', 'Miso Soup')
plot_trajectories(ax2, df_t, 'C_pct', 'Sigma_comp', '#ce0000', 'D', 'Miso+Tofu')

# 🌟 PIDが信じている「純水の理想モデル（直線）」を黒点線で引く
z_ideal = np.polyfit(df_w['C_pct'], df_w['Sigma_comp'], 1)
p_ideal = np.poly1d(z_ideal)
x_full = np.linspace(-0.1, 2.1, 100)
ax2.plot(x_full, p_ideal(x_full), color='black', linestyle='--', linewidth=1.8, zorder=4, label='PID Baseline (Water)')

# アノテーション：温度補正してもベースラインがズレるし、帯も太いまま！
ax2.annotate('Massive Model Mismatch', xy=(1.0, p_ideal(1.0)), xytext=(1.0, 10.5),
             ha='center', va='bottom', fontsize=8, fontweight='bold', color='black',
             arrowprops=dict(arrowstyle="<->", color="black", linewidth=1.2))

ax2.annotate('Residual Uncertainty\n(Temp compensation fails)', xy=(1.8, 16.5), xytext=(1.25, 22),
             ha='center', va='center', fontsize=8, fontweight='bold', color='#ce0000',
             arrowprops=dict(arrowstyle="->", color="#ce0000", linewidth=1.2))

ax2.set_title('(b) Compensated Conductivity $\\sigma_{comp}$', fontsize=10, fontweight='bold')
ax2.set_xlabel('True Salinity Concentration $C$ [%]')
ax2.set_ylabel('Compensated EC [mS/cm]')
ax2.grid(True, linestyle=':', alpha=0.6)
ax2.set_xlim(-0.05, 2.0)
ax2.set_ylim(-2, 25)
ax2.legend(loc='upper left', fontsize=7.5, framealpha=0.9, edgecolor='black')

filename = 'Fig2_Raw_vs_Compensated.png'
plt.savefig(filename, dpi=300, bbox_inches='tight')
plt.savefig('Fig2_Raw_vs_Compensated.pdf', bbox_inches='tight')

print(f"完了！最強の二段構えグラフを作成しました: {filename}")
plt.show()