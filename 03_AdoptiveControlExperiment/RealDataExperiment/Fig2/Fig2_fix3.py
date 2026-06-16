import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
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
# 2. データ処理とフィッティング関数
# ==========================================
T_room = 24.6  # 室温
a_temp = 0.02  # 温度係数
miso_initial_salt = 20.0 * 0.087  # 味噌由来の初期塩分(1.74g)

def process_and_fit(data_text, m_base_liquid, initial_salt):
    # スペース区切りのテキストをDataFrameとして読み込む
    df = pd.read_csv(io.StringIO(data_text.strip()), sep=r'\s+', skiprows=1, names=['Trial', 'C_g', 'T_C', 'Sigma'])
    df = df.dropna()
    
    # 厳密な塩分濃度と温度補正済み導電率の計算
    x_data = ((df['C_g'] + initial_salt) / (m_base_liquid + df['C_g'])) * 100
    y_data = df['Sigma'] / (1 + a_temp * (df['T_C'] - T_room))
    
    # 切片を通るフィッティング計算
    mask_zero = df['C_g'] == 0
    x_zero = x_data[mask_zero].mean()
    y_zero = y_data[mask_zero].mean()
    
    x_diff = x_data - x_zero
    y_diff = y_data - y_zero
    
    a = np.sum(x_diff * y_diff) / np.sum(x_diff**2)
    b = y_zero - a * x_zero
    
    # 決定係数
    y_pred = a * x_data + b
    ss_res = np.sum((y_data - y_pred) ** 2)
    ss_tot = np.sum((y_data - y_data.mean()) ** 2)
    r_squared = 1 - (ss_res / ss_tot)
    
    return x_data.values, y_data.values, a, b, r_squared

# データの処理実行
x1, y1, a1, b1, r2_1 = process_and_fit(data_env1, m_base_liquid=600.0, initial_salt=0.0)
x2, y2, a2, b2, r2_2 = process_and_fit(data_env2, m_base_liquid=600.0, initial_salt=miso_initial_salt)
x3, y3, a3, b3, r2_3 = process_and_fit(data_env3, m_base_liquid=500.0, initial_salt=miso_initial_salt)

# ==========================================
# 3. グラフの描画 (幅8cm, 2段構成)
# ==========================================
fig_width_inch = 8.0 / 2.54
fig_height_inch = 10 / 2.54 

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix' 
plt.rcParams['font.size'] = 9
plt.rcParams['axes.titlesize'] = 11
plt.rcParams['axes.labelsize'] = 9

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(fig_width_inch, fig_height_inch), gridspec_kw={'hspace': 0.8})

# --- 上段 (ax1): 純水だけのフィッティング ---
x_fit = np.linspace(-0.1, 1.5, 100)
ax1.scatter(x1, y1, color='black', marker='o', s=30, facecolors='none', 
            edgecolors="#262d32", alpha=0.6, linewidths=0.8, label=f'Measured Data')
ax1.plot(x_fit, a1 * x_fit + b1, color='#0095ff', linestyle='-', linewidth=1.5, 
         label=f'Linear Fit ($R^2={r2_1:.3f}$)', zorder=5)

ax1.set_title('(a) Sensor Calibration (Pure Water)', fontweight='bold', loc='center', pad=8)
ax1.set_xlabel(r'Salinity Concentration $C(t)$ [%]')
ax1.set_ylabel(r'$\sigma_{\text{comp}}$ [mS/cm]')
ax1.set_xlim(0, 1.4)
ax1.set_ylim(0, 18)
ax1.grid(True, linestyle=':', alpha=0.5)
ax1.legend(loc='upper left', fontsize=8, framealpha=0.9, edgecolor='black')

# --- 下段 (ax2): C=1.0% 時の導電率のズレ(%) ---
y1_at_1 = a1 * 1.0 + b1  
y2_at_1 = a2 * 1.0 + b2  
y3_at_1 = a3 * 1.0 + b3  

diff_miso = ((y2_at_1 - y1_at_1) / y1_at_1) * 100
diff_tofu = ((y3_at_1 - y1_at_1) / y1_at_1) * 100

categories = ['Miso Soup', 'Miso + Tofu']
values = [diff_miso, diff_tofu]
colors = ['#faaf00', '#00ff22']

x_pos = [0.28, 0.72]  # 2つのバーの中心位置（これで距離が 0.3 に縮まります）

# categories ではなく x_pos を渡して描画します
bars = ax2.bar(x_pos, values, color=colors, alpha=0.8, width=0.2, edgecolor='black', linewidth=1.0)

# 💡 手動指定した座標に、文字ラベルを貼り付けます
ax2.set_xticks(x_pos)
ax2.set_xticklabels(categories)

# 💡 グラフ全体の表示範囲を 0.0 〜 1.0 にして、左右の余白を整えます
ax2.set_xlim(0, 1.0)

ax2.set_title('(b) Conductivity Deviation at $C=1.0\%$', fontweight='bold', loc='center', pad=8)
# （これ以下の ax2.set_ylabel や ax2.axhline などはそのまま変更なしでOKです）
ax2.set_ylabel('Deviation vs. Water [%]')
ax2.grid(True, axis='y', linestyle=':', alpha=0.5)
ax2.set_ylim(0, max(values) * 1.3)

for bar in bars:
    yval = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2.0, yval + 0.5, f'+{yval:.1f}%', 
             ha='center', va='bottom', fontsize=9, fontweight='bold', color='black')

ax2.axhline(0, color='black', linewidth=1.0)

plt.tight_layout()

# ==========================================
# 4. 画像の保存
# ==========================================
plt.savefig('Fig2_8cm_2rows.png', dpi=600, bbox_inches='tight')  
plt.savefig('Fig2_8cm_2rows.pdf', bbox_inches='tight')           

plt.show()