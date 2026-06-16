import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import io

# ==========================================
# 1. データの準備（ここに直接データをコピペしてください）
# ==========================================
# 【水】のパラメータデータ
# ※例として適当な数値を入れています。途中で変更がなければ1行だけでもOKです。
data_water = """
Step    Alpha   Beta
1 8.1013	0.155986
"""

# 【味噌汁】のパラメータデータ
data_miso = """
Step    Alpha   Beta
1 8.1013	0.155
2 8.091153	0.138578
3 8.091153	0.138578
4 8.120508	0.157441
5 8.167884	0.17445
6 8.236687	0.185109
7 8.312784	0.185109
8 8.410188	0.185109
9 8.478678	0.185109
10 8.529858	0.185109
"""

# 【味噌汁＋豆腐】のパラメータデータ
data_tofu = """
Step    Alpha   Beta
1 8.1013	0.155
2 8.057168	0.095835
3 8.044991	0.084992
4 8.063935	0.095222
5 8.126473	0.112613
6 8.112747	0.112613
7 8.237057	0.112613
"""

# ==========================================
# 2. データ処理：24ステップまでの自動引き伸ばし関数
# ==========================================
def parse_and_extend(data_text, max_step=24):
    # テキストを読み込み、Stepをインデックス（基準）にする
    df = pd.read_csv(io.StringIO(data_text.strip()), sep=r'\s+')
    df = df.set_index('Step')
    
    # 1〜24までの新しいステップ軸を作成
    new_index = np.arange(1, max_step + 1)
    
    # 軸を24まで拡張し、空いた部分を「1つ前の数値」で自動的に埋める (ffill)
    # 最初の行がない場合は「後の数値」で埋める (bfill)
    df_extended = df.reindex(new_index).ffill().bfill()
    
    return df_extended

# 各データを処理
df_water = parse_and_extend(data_water)
df_miso  = parse_and_extend(data_miso)
df_tofu  = parse_and_extend(data_tofu)

# ==========================================
# 3. グラフの描画 (幅8cm, 2段構成)
# ==========================================
fig_width_inch = 8.0 / 2.54
fig_height_inch = 10.0 / 2.54  # 少し縦長に

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix' 
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 9

# 上下のグラフを作成（X軸を共有する）
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(fig_width_inch, fig_height_inch), 
                               sharex=True, gridspec_kw={'hspace': 0.8})

steps = df_water.index
colors = {'Water': "#0095ff", 'Miso': "#faaf00", 'Tofu': "#00ff22"}
lw = 1.5

# --- 上段 (ax1): Alpha の推移 ---
# 💡 パラメータの推移なので、階段状グラフ(step)を使います
ax1.step(steps, df_water['Alpha'], label='Pure Water', color=colors['Water'], linewidth=lw, where='post')
ax1.step(steps, df_miso['Alpha'],  label='Miso Soup',  color=colors['Miso'],  linewidth=lw, where='post')
ax1.step(steps, df_tofu['Alpha'],  label='Miso + Tofu',color=colors['Tofu'],  linewidth=lw, where='post')

ax1.set_ylabel(r'$\alpha$')

# 💡 以下の2行を追加して、X軸のラベルと数字を強制的に表示させます
ax1.set_xlabel('Step')
ax1.tick_params(labelbottom=True) 
ax1.set_ylim(7.9,8.6)  # Alphaの値が8.0前後なので、少し余裕を持たせた範囲に設定

ax1.set_title(r'(a) Parameter $\alpha$ - Step', fontweight='bold', loc='center', pad=8)
ax1.grid(True, linestyle=':', alpha=0.5)
ax1.set_title(r'(a) Parameter $\alpha$ - Step', fontweight='bold', loc='center', pad=8)
ax1.grid(True, linestyle=':', alpha=0.5)

# --- 下段 (ax2): Beta の推移 ---
ax2.step(steps, df_water['Beta'], color=colors['Water'], linewidth=lw, where='post')
ax2.step(steps, df_miso['Beta'],  color=colors['Miso'],  linewidth=lw, where='post')
ax2.step(steps, df_tofu['Beta'],  color=colors['Tofu'],  linewidth=lw, where='post')

ax2.set_ylabel(r'$\beta$')
ax2.set_xlabel('Step')
ax2.set_ylim(0.07, 0.2)
ax2.set_title(r'(b) Parameter $\beta$ - Step', fontweight='bold', loc='center', pad=8)
ax2.grid(True, linestyle=':', alpha=0.5)

# --- X軸と全体の装飾 ---
# 1〜24の範囲に固定し、メモリを綺麗に打つ
ax2.set_xlim(1, 24)
ax2.set_xticks([1, 6, 12, 18, 24])

# 図全体の上部に凡例を配置

plt.tight_layout()

plt.subplots_adjust(bottom=0.2)

# 💡 凡例を一番下（lower center）に配置する
fig.legend(['Pure Water', 'Miso Soup', 'Miso + Tofu'], 
           loc='lower center', 
           bbox_to_anchor=(0.5, 0.02), # Y座標を一番下スレスレ（0.02）に設定
           ncol=3, 
           fontsize=9, 
           frameon=False)

# ==========================================
# 4. 画像の保存
# ==========================================
plt.savefig('Fig8.png', dpi=600, bbox_inches='tight')
plt.savefig('Fig8.pdf', bbox_inches='tight')

plt.show()