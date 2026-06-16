import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ==========================================
# 1. データの準備（エクセルデータより抽出・パーセント変換）
# ==========================================
labels = ['Pure Water', 'Miso Soup', 'Miso Soup + Tofu']

# 各制御手法のエラー率（割合を % に直すために 100 を掛けます）
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
plt.rcParams['legend.fontsize'] = 12

fig, ax = plt.subplots(figsize=(fig_width_inch, fig_height_inch))

# --- グループ化された棒グラフの位置計算 ---
x = np.arange(len(labels))  # [0, 1, 2]
width = 0.25                # 棒の幅

x_ol = x - width
x_pid = x
x_str = x + width

# --- 💡 新しいデザイン設定（より濃く、説得力のあるカラー） ---
color_ol = '#7f7f7f'   # 濃い緑 (Open-Loop)
color_pid = '#1f77b4'#1f77b4'  # 濃い青 (PID Control)
color_str = '#c00000'  # 強い赤 (STR Adaptive - 一番目立たせたいので赤に変更)

# 棒グラフの透明度を上げて、色をハッキリ出す（0.6 -> 0.85）
alpha_bar = 0.85        
lw_main = 2.5          # 折れ線の太さ
ms = 10                # マーカーのサイズ

# ==========================================
# 3. 描画（バーグラフ ＋ 折れ線グラフ）
# ==========================================
# ① バーグラフ
ax.bar(x_ol, err_ol, width, label='Open-Loop (Bar)', color=color_ol, alpha=alpha_bar, edgecolor='black', linewidth=1.0)
ax.bar(x_pid, err_pid, width, label='PID Control (Bar)', color=color_pid, alpha=alpha_bar, edgecolor='black', linewidth=1.0)
ax.bar(x_str, err_str, width, label='STR Adaptive (Bar)', color=color_str, alpha=alpha_bar, edgecolor='black', linewidth=1.0)

# ② 折れ線グラフ
#line_color_ol = '#7f7f7f' # 線用の濃い緑
#ax.plot(x_ol, err_ol, color=line_color_ol, marker='o', markersize=ms, linestyle='-', linewidth=lw_main, zorder=4)
#ax.plot(x_pid, err_pid, color=color_pid, marker='s', markersize=ms, linestyle='-', linewidth=lw_main, zorder=4)
#ax.plot(x_str, err_str, color=color_str, marker='D', markersize=ms+4, linestyle='-', linewidth=lw_main, zorder=4)

# --- 💡 4. エクセルの数値を直書きする処理 ---
# y_offset は、点からどれくらい上に数字を浮かせるかの調整値です
# --- エクセルの数値を直書きする処理 ---
y_offset = 1.5 
# 💡 ここを追加！文字を右に少しズラすための調整値（0.02〜0.04くらいで一番綺麗に見える数字を探してください）
x_shift = 0.025 

def add_value_labels(x_coords, y_coords, text_color):
    for x_val, y_val in zip(x_coords, y_coords):
        # 💡 x_val に x_shift を足して、描画位置を右にズラす
        ax.text(x_val + x_shift, y_val + y_offset, f"{y_val:.1f}%", 
                ha='center', va='bottom', fontsize=11, 
                fontweight='bold', color=text_color, zorder=5,
                #bbox=dict(facecolor='white', edgecolor='none', alpha=0.85, boxstyle='round,pad=0.2')
                )

# 文字の色はバーの色に合わせて少し暗くする
add_value_labels(x_ol, err_ol, '#4d4d4d')   
add_value_labels(x_pid, err_pid, '#124870') 
add_value_labels(x_str, err_str, '#8b0000')
# ==========================================
# 5. 装飾とレイアウト調整
# ==========================================
ax.set_ylabel('Steady-State Relative Error [%]')
ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=14)

# 💡 数値ラベルがグラフ上端にぶつからないように最大値を少し広げる
ax.set_ylim(0, 65)
ax.grid(True, axis='y', linestyle=':', alpha=0.6)

# 下部に「スープが複雑になる」という矢印を追加
ax.annotate('', xy=(0.9, -0.15), xycoords='axes fraction', xytext=(0.1, -0.15),
            arrowprops=dict(arrowstyle="->", color="black", lw=2))
ax.text(0.5, -0.22, 'Increasing Soup Complexity', transform=ax.transAxes, 
        ha='center', va='center', fontsize=14, fontweight='bold')

# 凡例の設定
#legend_elements = [
    #Line2D([0], [0], color=line_color_ol, marker='o', linestyle='-', lw=lw_main, markersize=ms, label='Open-Loop'),
    #Line2D([0], [0], color=color_pid, marker='s', linestyle='-', lw=lw_main, markersize=ms, label='PID Control'),
    #Line2D([0], [0], color=color_str, marker='D', linestyle='-', lw=lw_main, markersize=ms+4, label='STR (Adaptive)')
#]

# --- 凡例の設定（マーカーを合わせる） ---
# 💡 ここで Patch をインポートします
from matplotlib.patches import Patch

# 💡 線(Line2D)ではなく、四角い色面(Patch)を作って凡例にします
legend_elements = [
    Patch(facecolor=color_ol, edgecolor='black', alpha=alpha_bar, label='Open-Loop'),
    Patch(facecolor=color_pid, edgecolor='black', alpha=alpha_bar, label='PID Control'),
    Patch(facecolor=color_str, edgecolor='black', alpha=alpha_bar, label='STR (Adaptive)')
]
ax.legend(handles=legend_elements, loc='upper left', frameon=True, edgecolor='black', framealpha=1.0)

plt.tight_layout()
plt.subplots_adjust(bottom=0.25)

# 画像の保存
plt.savefig('Fig10_bar1.png', dpi=600, bbox_inches='tight')
plt.savefig('Fig10_bar1.pdf', bbox_inches='tight')

plt.show()