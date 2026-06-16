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
# 2. データ処理とフィッティングの自動化関数
# ==========================================
T_room = 24.6  # 室温
a_temp = 0.02  # 温度係数

# 味噌由来の初期塩分 (20g * 8.7%)
miso_initial_salt = 20.0 * 0.087  # 1.74g

def process_and_fit(data_text, m_base_liquid, initial_salt):
    df = pd.read_csv(io.StringIO(data_text.strip()), sep=r'\s+', skiprows=1, names=['Trial', 'C_g', 'T_C', 'Sigma'])
    df = df.dropna()
    
    # X軸: 厳密な塩分濃度 C(%) の計算
    x_data = ((df['C_g'] + initial_salt) / (m_base_liquid + df['C_g'])) * 100
    
    # Y軸: 温度補正済み導電率 σ_comp の計算
    y_data = df['Sigma'] / (1 + a_temp * (df['T_C'] - T_room))
    
    # ─── ★ここを修正：実際のスタート点 (x_zero, y_zero) を必ず通る直線フィッティング ───
    # 塩0g（C_g == 0）のときの、実際の「平均濃度」と「平均導電率」を取得
    mask_zero = df['C_g'] == 0
    x_zero = x_data[mask_zero].mean()
    y_zero = y_data[mask_zero].mean()
    
    # (x_zero, y_zero) からの相対的な距離（ズレ）を計算
    x_diff = x_data - x_zero
    y_diff = y_data - y_zero
    
    # 実際のスタート点を通る条件での、最適な傾き a を逆算
    a = np.sum(x_diff * y_diff) / np.sum(x_diff**2)
    
    # Y軸の切片 b (X=0 のときの値) を物理モデルに合わせて逆算
    b = y_zero - a * x_zero
    # ─────────────────────────────────────────────────────────────────────────────────
    
    # 3. 決定係数 R^2 の算出
    y_pred = a * x_data + b
    ss_res = np.sum((y_data - y_pred) ** 2)
    ss_tot = np.sum((y_data - y_data.mean()) ** 2)
    r_squared = 1 - (ss_res / ss_tot)
    
    print(f"--- fitting results (Corrected Fixed Point) ---")
    print(f"Liquid Mass: {m_base_liquid}g, Initial Salt: {initial_salt}g")
    print(f"a (Sensitivity) = {a:.4f}, b (Baseline) = {b:.4f}, R^2 = {r_squared:.4f}\n")
    
    return x_data.values, y_data.values, a, b, r_squared


# 3つの環境のデータを処理
x1, y1, a1, b1, r2_1 = process_and_fit(data_env1, m_base_liquid=600.0, initial_salt=0.0)
x2, y2, a2, b2, r2_2 = process_and_fit(data_env2, m_base_liquid=600.0, initial_salt=miso_initial_salt)
x3, y3, a3, b3, r2_3 = process_and_fit(data_env3, m_base_liquid=500.0, initial_salt=miso_initial_salt)


# ==========================================
# 3. 論文用グラフの描画 (幅8cm, Times New Roman) ＋ 拡大図
# ==========================================
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

fig_width_inch = 8.0 / 2.54
fig_height_inch = fig_width_inch * 0.75 

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix' 
plt.rcParams['font.size'] = 9
plt.rcParams['legend.fontsize'] = 8

fig, ax = plt.subplots(figsize=(fig_width_inch, fig_height_inch))
x_fit = np.linspace(-0.1, 1.7, 100)

# --- 線の太さとマーカーの透明度を調整 ---
lw_main = 1.5   # 線を少し太くして強調
alpha_m = 0.4   # マーカーを薄くして線を邪魔しないようにする
z_line  = 5     # 線を前面に持ってくる (zorder)

# Env 1 (純水): 黒 / 丸 / 実線
scatter1 = ax.scatter(x1, y1, color='black', marker='o', s=20, facecolors='none', edgecolors='black', alpha=alpha_m)
line1, = ax.plot(x_fit, a1 * x_fit + b1, color='black', linestyle='-', linewidth=lw_main, zorder=z_line)

# Env 2 (水＋味噌): 赤 / 三角 / 破線（一点鎖線より破線の方が見やすいです）
scatter2 = ax.scatter(x2, y2, color='#d62728', marker='^', s=20, facecolors='none', edgecolors='#d62728', alpha=alpha_m)
line2, = ax.plot(x_fit, a2 * x_fit + b2, color='#d62728', linestyle='--', linewidth=lw_main, zorder=z_line)

# Env 3 (完全な味噌汁): 青 / 四角 / 点線（少し太め）
scatter3 = ax.scatter(x3, y3, color='#1f77b4', marker='s', s=20, facecolors='none', edgecolors='#1f77b4', alpha=alpha_m)
line3, = ax.plot(x_fit, a3 * x_fit + b3, color='#1f77b4', linestyle=':', linewidth=lw_main+0.5, zorder=z_line)

# --- 軸とグリッドの設定 ---
ax.set_xlabel(r'Salinity Concentration $C(t)$ [%]')
ax.set_ylabel('Compensated Conductivity\n' + r'$\sigma_{\text{comp}}$ [mS/cm]')
ax.set_xlim(0, 1.8)  
ax.set_ylim(0, 15)   
ax.grid(True, linestyle=':', alpha=0.6)

# --- 💡 テクニック：拡大図（Inset）を右下に追加 ---
# loc=4 は右下 (lower right) を意味します
axins = inset_axes(ax, width="35%", height="35%", loc=4, borderpad=2)

# 拡大図にも同じデータを描画
axins.plot(x_fit, a1 * x_fit + b1, color='black', linestyle='-', linewidth=lw_main, zorder=z_line)
axins.plot(x_fit, a2 * x_fit + b2, color='#d62728', linestyle='--', linewidth=lw_main, zorder=z_line)
axins.plot(x_fit, a3 * x_fit + b3, color='#1f77b4', linestyle=':', linewidth=lw_main+0.5, zorder=z_line)
axins.scatter(x1, y1, color='black', marker='o', s=10, facecolors='none', edgecolors='black', alpha=alpha_m)
axins.scatter(x2, y2, color='#d62728', marker='^', s=10, facecolors='none', edgecolors='#d62728', alpha=alpha_m)
axins.scatter(x3, y3, color='#1f77b4', marker='s', s=10, facecolors='none', edgecolors='#1f77b4', alpha=alpha_m)

# 拡大する範囲を指定（線のばらつきが分かりやすい 0.2〜0.6% 付近）
axins.set_xlim(0.8, 1.1)
axins.set_ylim(7, 11)
axins.tick_params(labelsize=7)
axins.grid(True, linestyle=':', alpha=0.4)

# 拡大元の範囲と拡大図を結ぶ線を引く
mark_inset(ax, axins, loc1=1, loc2=3, fc="none", ec="0.5", alpha=0.5)

# --- 凡例の設定 ---
#legend_handles = [(scatter1, line1), (scatter2, line2), (scatter3, line3)]
#legend_labels = [
    #f'Pure Water ($R^2={r2_1:.3f}$)',
    #f'Miso Soup ($R^2={r2_2:.3f}$)',
    #f'Miso + Tofu ($R^2={r2_3:.3f}$)'
#]

# ax.legend(
    #handles=legend_handles,
    #labels=legend_labels,
    #ncol=1, 
    #loc='upper left',     
    #frameon=True, 
    #edgecolor='black', 
    #fancybox=False, 
    #handlelength=2.5,
    #fontsize=7.5,         
    #labelspacing=0.35     
    #)

plt.tight_layout()
# ==========================================
# 4. 画像の保存 (PNG, SVG, PDF)
# ==========================================
plt.savefig('calibration_curves.png', dpi=600, bbox_inches='tight')  
plt.savefig('calibration_curves.svg', bbox_inches='tight')           
plt.savefig('calibration_curves.pdf', bbox_inches='tight')           

plt.show()