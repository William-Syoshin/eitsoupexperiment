import matplotlib.pyplot as plt
import numpy as np

# --- 論文用 8cm幅 フォーマット設定 ---
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.labelsize': 10,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 8,
    'lines.linewidth': 1.5,
})

# ==========================================================
# ✍️ 手打ち用データ入力エリア
# ==========================================================
# 横軸：塩の量 [g]
salt_amounts = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

# 縦軸①：ロボットの想定（水のみの導電率）
sigma_water = np.array([0.259, 3.18, 5.34, 7.39, 9.43, 11.6, 13.9])

# 縦軸②：現実の環境（水＋ポテトの導電率）
sigma_potato = np.array([0.25, 3.64, 6.11, 10.1, 12.82, 14.95, 18.3])
# ==========================================================

# グラフの作成 (幅8cm, 高さ6cm に設定)
fig, ax = plt.subplots(figsize=(8 / 2.54, 6 / 2.54))

water_color = '#0000FF'   # 水（想定）の色：青
soup_color = '#FF8C00'    # スープ（現実）の色：オレンジ

# 1. ロボットの想定（水のみ）：青の実線 ＋ 丸い点
ax.plot(salt_amounts, sigma_water, color=water_color, ls='-', label='Assumption (Water)')
ax.scatter(salt_amounts, sigma_water, color=water_color, s=40, edgecolor='black', zorder=3)

# 2. 現実の環境（ポテト入り）：オレンジの破線 ＋ 三角の点
ax.plot(salt_amounts, sigma_potato, color=soup_color, ls='--', label='Actual (Potato)')
ax.scatter(salt_amounts, sigma_potato, color=soup_color, marker='^', s=50, edgecolor='black', zorder=3)

# 3. 感知誤差（モデル不一致）を強調する矢印
target_idx = 3 # 3gの地点に矢印を引く場合
start_pt = (salt_amounts[target_idx], sigma_water[target_idx])
end_pt = (salt_amounts[target_idx], sigma_potato[target_idx])

ax.annotate('', xy=end_pt, xytext=start_pt,
             arrowprops=dict(facecolor='red', edgecolor='red', shrink=0.05, width=1.5, headwidth=6))

# 「感知誤差」のラベル（グラフの線と被らないように左側に配置調整）
ax.text(salt_amounts[target_idx] - 0.2, (sigma_water[target_idx] + sigma_potato[target_idx])/2, 
         'Sensing Error\n(Mismatch)', color='red', fontsize=9, fontweight='bold', ha='right', va='center')

# 軸ラベルの設定
ax.set_xlabel('Salt Amount [g]')
ax.set_ylabel('Conductivity $\sigma$ [mS/cm]')

# 凡例を枠付きで配置
ax.legend(loc='upper left', frameon=True, edgecolor='black')
ax.grid(alpha=0.3)

# 余白の自動調整
plt.tight_layout()

# 保存（SVGと高解像度PNG）
plt.savefig('Fig2_Sensing_Characteristics_8cm.svg', format='svg')
plt.savefig('Fig2_Sensing_Characteristics_8cm.png', dpi=300)
plt.show()