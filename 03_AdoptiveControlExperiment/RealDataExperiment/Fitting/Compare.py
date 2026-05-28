import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import io

# ==========================================
# 1. 生データのコピペエリア
# ==========================================

# 【Env 1: 純水 (水600g)】
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
2   0   49.33   0.259
2   1   48.45   2.030
2   2   49.98   4.160
2   3   50.59   5.610
2   4   51.62   7.200
2   5   49.23   8.730
2   6   46.44   10.21
2   8   51.62   11.51
2   9   53.59   12.80
3   0   49.33   0.218
3   1   50.79   2.820
3   2   50.32   5.500
3   3   49.74   7.120
3   4   51.62   9.220
3   5   51.62   11.20
3   6   50.35   12.61
3   7   49.35   13.77
3   8   48.55   16.55
"""

# 【Env 2: 水＋miso (水580g + 味噌20g)】
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

# 【Env 3: 味噌汁 (水480g + 味噌20g + 豆腐100g)】
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
# 2. データ処理とフィッティングの自動化関数
# ==========================================
T_room = 24.6  # 室温
a_temp = 0.02  # 温度係数

# 味噌由来の初期塩分 (20g * 8.7%)
miso_initial_salt = 20.0 * 0.087  # 1.74g

def process_and_fit(data_text, m_base_liquid, initial_salt):
    # sep=r'\s+' : タブでもスペースでも、連続する空白をすべて区切り文字として処理する
    # skiprows=1 : 記号混じりのヘッダー行を無視する
    # names=[...] : プログラム内で扱いやすい安全な列名を強制的に割り当てる
    df = pd.read_csv(io.StringIO(data_text.strip()), sep=r'\s+', skiprows=1, names=['Trial', 'C_g', 'T_C', 'Sigma'])
    df = df.dropna()
    
    # X軸: 厳密な塩分濃度 C(%) の計算
    # 分子: 加えた塩 C_g + 初期塩分
    # 分母: ベース液相 + 加えた塩 C_g
    x_data = ((df['C_g'] + initial_salt) / (m_base_liquid + df['C_g'])) * 100
    
    # Y軸: 温度補正済み導電率 σ_comp の計算
    y_data = df['Sigma'] / (1 + a_temp * (df['T_C'] - T_room))
    
    # 線形回帰で a (傾き), b (切片), r_value (相関係数) を計算
    a, b, r_value, _, _ = linregress(x_data, y_data)
    r_squared = r_value**2
    
    print(f"--- fitting results ---")
    print(f"Liquid Mass: {m_base_liquid}g, Initial Salt: {initial_salt}g")
    print(f"a (Sensitivity) = {a:.4f}, b (Baseline) = {b:.4f}, R^2 = {r_squared:.4f}\n")
    
    return x_data.values, y_data.values, a, b, r_squared

# ↓↓↓ おそらくここが消えてしまっていました！ ↓↓↓
# 3つの環境のデータを処理 (物理モデルに基づいた質量定義)
# Env 1: 純水 (ベース液相 600g, 初期塩分 0g)
x1, y1, a1, b1, r2_1 = process_and_fit(data_env1, m_base_liquid=600.0, initial_salt=0.0)

# Env 2: 水580g + 味噌20g (ベース液相 600g, 初期塩分 1.74g)
x2, y2, a2, b2, r2_2 = process_and_fit(data_env2, m_base_liquid=600.0, initial_salt=miso_initial_salt)

# Env 3: 水480g + 味噌20g + 豆腐100g (豆腐は固形物として除外し、ベース液相 500g, 初期塩分 1.74g)
x3, y3, a3, b3, r2_3 = process_and_fit(data_env3, m_base_liquid=500.0, initial_salt=miso_initial_salt)
# ↑↑↑ ここまで ↑↑↑


# ==========================================
# 3. 論文用グラフの描画 (幅8cm, Times New Roman)
# ==========================================
fig_width_inch = 8.0 / 2.54
fig_height_inch = fig_width_inch * 0.75 

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix' 
plt.rcParams['font.size'] = 9
plt.rcParams['legend.fontsize'] = 8

fig, ax = plt.subplots(figsize=(fig_width_inch, fig_height_inch))
x_fit = np.linspace(-0.1, 1.7, 100) # X軸の描画範囲

# Env 1 (純水): 黒 / 丸 / 実線
scatter1 = ax.scatter(x1, y1, color='black', marker='o', s=20, facecolors='none', edgecolors='black', alpha=0.7)
line1, = ax.plot(x_fit, a1 * x_fit + b1, color='black', linestyle='-', linewidth=1.2)

# Env 2 (水＋味噌): 赤 / 三角 / 一点鎖線
scatter2 = ax.scatter(x2, y2, color='#d62728', marker='^', s=20, facecolors='none', edgecolors='#d62728', alpha=0.7)
line2, = ax.plot(x_fit, a2 * x_fit + b2, color='#d62728', linestyle='-.', linewidth=1.2)

# Env 3 (完全な味噌汁): 青 / 四角 / 破線
scatter3 = ax.scatter(x3, y3, color='#1f77b4', marker='s', s=20, facecolors='none', edgecolors='#1f77b4', alpha=0.7)
line3, = ax.plot(x_fit, a3 * x_fit + b3, color='#1f77b4', linestyle='--', linewidth=1.2)

# --- 軸のラベルと表示範囲の設定 ---
ax.set_xlabel(r'Salinity Concentration $C(t)$ [%]')
ax.set_ylabel('Compensated Conductivity\n' + r'$\sigma_{\text{comp}}$ [mS/cm]')

ax.set_xlim(0, 1.8)  
ax.set_ylim(0, 25)   

ax.grid(True, linestyle=':', alpha=0.6)

# --- 凡例の微調整 ---
legend_handles = [(scatter1, line1), (scatter2, line2), (scatter3, line3)]
legend_labels = [
    f'Pure Water ($R^2={r2_1:.3f}$)',
    f'Miso Soup ($R^2={r2_2:.3f}$)',
    f'Miso soup + Tofu ($R^2={r2_3:.3f}$)'
]

ax.legend(
    handles=legend_handles,
    labels=legend_labels,
    ncol=1, 
    loc='upper left',     
    frameon=True, 
    edgecolor='black', 
    fancybox=False, 
    handlelength=2.5,
    fontsize=7.5,         
    labelspacing=0.35     
)

plt.tight_layout()

# ==========================================
# 4. 画像の保存 (PNG, SVG, PDF)
# ==========================================
plt.savefig('calibration_curves.png', dpi=600, bbox_inches='tight')  
plt.savefig('calibration_curves.svg', bbox_inches='tight')           
plt.savefig('calibration_curves.pdf', bbox_inches='tight')           

plt.show()