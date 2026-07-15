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
# 2. データ抽出と「純水を基準としたズレ」の計算
# ==========================================
def extract_raw_data(data_text, m_base_liquid, initial_salt):
    df = pd.read_csv(io.StringIO(data_text.strip()), sep=r'\s+', skiprows=1, names=['Trial', 'C_g', 'T_C', 'Sigma'])
    df = df.dropna()
    df['C_pct'] = ((df['C_g'] + initial_salt) / (m_base_liquid + df['C_g'])) * 100
    return df

miso_initial_salt = 20.0 * 0.087

df_w = extract_raw_data(data_env1, 600.0, 0.0)               
df_m = extract_raw_data(data_env2, 600.0, miso_initial_salt) 
df_t = extract_raw_data(data_env3, 500.0, miso_initial_salt) 

# 🌟 【コアロジック】生の純水データだけで「無理やり」直線を引く
z_ideal = np.polyfit(df_w['C_pct'], df_w['Sigma'], 1)
p_ideal = np.poly1d(z_ideal)

# 全てのデータを「純水の直線からのズレ（偏差）」に変換
def calc_deviation_trials(df):
    df['Deviation'] = df['Sigma'] - p_ideal(df['C_pct'])
    trials = []
    for trial_num in df['Trial'].unique():
        trial_df = df[df['Trial'] == trial_num]
        trials.append((trial_df['C_pct'].values, trial_df['Deviation'].values))
    return trials, df['C_pct'].values, df['Deviation'].values

trials_w, all_c_w, dev_w = calc_deviation_trials(df_w)
trials_m, all_c_m, dev_m = calc_deviation_trials(df_m)
trials_t, all_c_t, dev_t = calc_deviation_trials(df_t)

# ==========================================
# 3. グラフの描画（Model Mismatch & Chaos の可視化）
# ==========================================
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.size'] = 10

# 🌟 修正ポイント1: constrained_layout=True を「削除」し、高さを6.5cmに変更
fig, ax = plt.subplots(figsize=(8.0 / 2.54, 8 / 2.54))

# 基準線とノイズ帯
ax.axhline(0, color='black', linestyle='--', linewidth=1.5, zorder=2, label='Baseline Reference (Pure Water)')
std_w = np.std(dev_w)
ax.fill_between([-0.1, 2.1], -2*std_w, 2*std_w, color='#1f77b4', alpha=0.15, zorder=1, label='Expected Sensor Noise')

def plot_chaos_trajectories(ax, trials, all_c, all_dev, color, marker, label):
    z_dev = np.polyfit(all_c, all_dev, 1)
    p_dev = np.poly1d(z_dev)
    x_range = np.linspace(min(all_c), max(all_c), 100)
    
    res = all_dev - p_dev(all_c)
    std_res = np.std(res)
    
    ax.fill_between(x_range, p_dev(x_range) - 2*std_res, p_dev(x_range) + 2*std_res, color=color, alpha=0.15, zorder=1)
    
    for i, (x, y) in enumerate(trials):
        lbl = label if i == 0 else ""
        ax.plot(x, y, color=color, marker=marker, markersize=4, linestyle='-', linewidth=1.2, alpha=0.8, label=lbl, zorder=3)

# 味噌、味噌＋豆腐をプロット
plot_chaos_trajectories(ax, trials_m, all_c_m, dev_m, '#ff7f0e', 's', 'Miso Soup')
plot_chaos_trajectories(ax, trials_t, all_c_t, dev_t, '#ce0000', 'D', 'Miso Soup + Tofu')

# 軸ラベルと範囲設定
ax.set_xlabel('True Salinity $C$ [%]', fontsize=11)
ax.set_ylabel('EC Deviation \n $\\Delta\\sigma$ [mS/cm]', fontsize=11)
ax.set_xlim(0.2, 2.0)
ax.set_ylim(-2, 7) 
ax.grid(True, linestyle=':', alpha=0.6)
ax.tick_params(axis='both', which='major', labelsize=10)


# ==========================================
# 🌟 凡例の並び順と枠線の設定
# ==========================================
handles, labels = ax.get_legend_handles_labels()

new_handles = [handles[0], handles[2], handles[1], handles[3]]
new_labels  = [labels[0], labels[2], labels[1], labels[3]]

# frameon=False で黒い枠線を消去し、位置を下部に設定
ax.legend(new_handles, new_labels, loc='upper center', bbox_to_anchor=(0.5, -0.28), 
          ncol=2, fontsize=9, frameon=False, columnspacing=1.0)

# 🌟 修正ポイント2: 手動で余白を設定し、グラフが潰れるのを防ぐ（保存の直前に追加）
plt.subplots_adjust(bottom=0.35, left=0.18, right=0.95, top=0.9)

filename_png = 'Fig2_Raw_Deviation_8cm.png'

filename_pdf = 'Fig2_Raw_Deviation_8cm.pdf'
plt.savefig(filename_png, dpi=300, bbox_inches='tight')
plt.savefig(filename_pdf, bbox_inches='tight')

print(f"完了！グラフ本体のバランスを復元し、凡例を綺麗に配置しました:\n  - {filename_png}\n  - {filename_pdf}")
plt.show()