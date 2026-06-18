import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ==========================================
# 1. データの準備（エクセルデータより抽出・パーセント変換）
# ==========================================
labels = ['Pure Water', 'Miso Soup', 'Miso Soup + Tofu']

err_ol = np.array([0.0, 0.29083018, 0.58623678]) * 100
err_pid = np.array([0.01149479, 0.20420506, 0.13296925]) * 100
err_str = np.array([0.02407179, 0.16519775, 0.05508457]) * 100

# ==========================================
# 2. グラフの設定 (幅15cm, Times New Roman)
# ==========================================
fig_width_inch = 15.0 / 2.54
fig_height_inch = fig_width_inch * 0.65

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix' 
plt.rcParams['font.size'] = 14
plt.rcParams['legend.fontsize'] = 11

fig, ax = plt.subplots(figsize=(fig_width_inch, fig_height_inch))

# --- 折れ線グラフ用のX座標（すべて同じ位置に揃える） ---
x = np.arange(len(labels))  

# --- デザイン設定 ---
color_ol = '#7f7f7f'   # 濃いグレー (Open-Loop)
color_pid = '#1f77b4'  # 濃い青 (PID Control)
color_str = '#c00000'  # 強い赤 (STR Adaptive)

lw_main = 2.5          # 折れ線の太さ
ms = 10                # マーカーのサイズ

# ==========================================
# 3. 描画（折れ線グラフのみ）
# ==========================================
ax.plot(x, err_ol, color=color_ol, marker='o', markersize=ms, linestyle='-', linewidth=lw_main, zorder=4)
ax.plot(x, err_pid, color=color_pid, marker='s', markersize=ms, linestyle='-', linewidth=lw_main, zorder=4)
ax.plot(x, err_str, color=color_str, marker='D', markersize=ms+2, linestyle='-', linewidth=lw_main, zorder=4)

# ==========================================
# 4. エクセルの数値を直書きする処理
# ==========================================

# 💡 各点に対する文字のズレを個別に指定します。(X方向のズレ, Y方向のズレ) ※単位はピクセル
# リストの中身は [Pure Waterでのズレ, Miso Soupでのズレ, Miso+Tofuでのズレ] です。

# Open-Loop (グレー): Pure Water(0%) は一番下なので、文字を下(-15)に逃がす
offset_ol  = [(0, 19), (0, 30), (0, -18)]

# PID Control (青): 基本は上(15)に配置
offset_pid = [(0, 32), (0, 10), (0, 15)]

# STR Adaptive (赤): Pure Water(2.4%)は青と被るのでさらに上(30)へ。他は青と被らないよう下(-15)へ逃がす
offset_str = [(0, 45), (0, -18), (0, -18)]

def add_value_labels_smart(x_coords, y_coords, text_color, offsets):
    for i, (x_val, y_val) in enumerate(zip(x_coords, y_coords)):
        ax.annotate(f"{y_val:.1f}%",
                    xy=(x_val, y_val),        # 基準となる点の座標
                    xytext=offsets[i],        # 設定したオフセット（ズレ）
                    textcoords="offset points", # ズレの単位をピクセルにする設定
                    ha='center', va='center', # 💡 ここで幾何学的に「完全なド真ん中」に揃えます
                    fontsize=11, fontweight='bold', color=text_color, zorder=5,
                    #bbox=dict(facecolor='white', edgecolor='none', alpha=0.85, boxstyle='round,pad=0.2')
                    )

# 各ラインに個別のオフセットを適用して描画
add_value_labels_smart(x, err_ol, '#4d4d4d', offset_ol)
add_value_labels_smart(x, err_pid, '#124870', offset_pid)
add_value_labels_smart(x, err_str, '#8b0000', offset_str)
# ==========================================
# 5. 装飾とレイアウト調整
# ==========================================
ax.set_ylabel('Steady-State Relative Error [%]')
ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=14)

# 💡 折れ線が左右の枠にぶつからないようにX軸に余白を作る
ax.set_xlim(-0.3, 2.3)
# Y軸も0のラベルが見切れないように少し下から始める
ax.set_ylim(-3, 62)
ax.grid(True, axis='y', linestyle=':', alpha=0.6)

# 下部に「スープが複雑になる」という矢印を追加
ax.annotate('', xy=(0.9, -0.15), xycoords='axes fraction', xytext=(0.1, -0.15),
            arrowprops=dict(arrowstyle="->", color="black", lw=2))
ax.text(0.5, -0.22, 'Increasing Soup Complexity', transform=ax.transAxes, 
        ha='center', va='center', fontsize=14, fontweight='bold')

# --- 凡例の設定（線とマーカーに戻す） ---
legend_elements = [
    Line2D([0], [0], color=color_ol, marker='o', linestyle='-', lw=lw_main, markersize=ms, label='Open-Loop'),
    Line2D([0], [0], color=color_pid, marker='s', linestyle='-', lw=lw_main, markersize=ms, label='PID Control'),
    Line2D([0], [0], color=color_str, marker='D', linestyle='-', lw=lw_main, markersize=ms+2, label='Adaptive Tasting Control')
]
ax.legend(handles=legend_elements, loc='upper left', frameon=True, edgecolor='black', framealpha=1.0)

plt.tight_layout()
plt.subplots_adjust(bottom=0.25)

# 画像の保存
plt.savefig('Fig10_line.png', dpi=600, bbox_inches='tight')
plt.savefig('Fig10_line.pdf', bbox_inches='tight')

plt.show()