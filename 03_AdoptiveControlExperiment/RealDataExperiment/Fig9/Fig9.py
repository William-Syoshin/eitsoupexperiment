import matplotlib.pyplot as plt

# ==============================================================================
# 1. 実験データの入力エリア（ここに24ステップ分の数値を直接書き込んでください）
# ==============================================================================
# ※現在はサンプルとしてすべて「0.5」や「1.0」を入れています。実際の数値に書き換えてください。
# ※エクセルから数値をコピーして、カンマ `,` で区切って並べればOKです。

# --- ① 水 (Water) のデータ（各24要素） ---
water_ol  = [1.0]*24  # Open-Loop のデータ (例: [1.0, 1.0, 1.0, ...])
w_pid_raw = [-0.000432759, 0.104614227, 0.215700957, 0.319640602, 0.422171288, 0.51621772, 0.607208249, 0.722747691, 0.80761029, 0.890491235, 0.986650062, 1.039532742]
water_pid = w_pid_raw + [w_pid_raw[-1]] * (24 - len(w_pid_raw))
w_str_raw = [0.000221222,
0.066793307,
0.153404914,
0.254087459,
0.340925965,
0.437549218,
0.509476639,
0.614398629,
0.703287245,
0.780267015,
0.847584054,
0.894943459,
0.991438219,    
1.005897544,
1.010343698,
1.00445099,
1.001299333,
1.01810388,
]
water_str = w_str_raw + [w_str_raw[-1]] * (24 - len(w_str_raw))

# --- ② 味噌汁 (Miso Soup) のデータ（各24要素） ---
miso_ol   = [1.283088804,
1.283088804,
1.291585081,
1.291585081,
1.250544391,
1.250544391,
1.289057649,
1.289057649,
1.305827402,
1.305827402,
1.308012953,
1.308012953,
1.323967322,
1.323967322,
1.328991554,
1.328991554,
1.298341852,
1.298341852,
1.278377518,
1.278377518,
1.266400051,
1.266400051,
1.265767617,
1.265767617]
miso_pid_raw = [0.311151154,
0.377470922,
0.462648067,
0.579607873,
0.676456697,
0.770202005,
0.764442091,
0.786995848,
0.788612309]
miso_pid = miso_pid_raw + [miso_pid_raw[-1]] * (24 - len(miso_pid_raw))
miso_str_raw = [0.270914512,
0.338354573,
0.418397606,
0.507443972,
0.594943023,
0.685628138,
0.771113971,
0.833399432,
0.834334036,
0.835270461]
miso_str = miso_str_raw + [miso_str_raw[-1]] * (24 - len(miso_str_raw))

# --- ③ 味噌＋豆腐 (Miso+Tofu) のデータ（各24要素） ---
tofu_ol   = [1.600138599,
1.600138599,
1.617406542,
1.617406542,
1.62473713,
1.62473713,
1.588498823,
1.588498823,
1.568930072,
1.568930072,
1.568022934,
1.568022934,
1.571202185,
1.571202185,
1.572796302,
1.572796302,
1.56718766,
1.56718766,
1.593171134,
1.593171134,
1.595719313,
1.595719313,
1.56703067,
1.56703067]
tofu_pid_raw = [0.496732049,
0.601499028,
0.74885993,
0.903602133,
0.918535909,
0.911149773,
0.846863981,
0.866354767]
tofu_pid = tofu_pid_raw + [tofu_pid_raw[-1]] * (24 - len(tofu_pid_raw))

tofu_str_raw  = [0.417202589,
0.469642388,
0.557690669,
0.654209423,
0.762905525,
0.804476759,
0.942606925]
tofu_str = tofu_str_raw + [tofu_str_raw[-1]] * (24 - len(tofu_str_raw))

# X軸のステップ（1から24まで自動生成）
steps = list(range(1, 25))

# ==============================================================================
# 2. 全体のフォントと文字サイズの設定 (Times New Roman, 9〜12)
# ==============================================================================
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 12              # グラフタイトルのサイズ
plt.rcParams['axes.labelsize'] = 11              # 軸ラベル（Step, C_real [%]）のサイズ
plt.rcParams['xtick.labelsize'] = 9              # X軸目盛り数字のサイズ
plt.rcParams['ytick.labelsize'] = 9              # Y軸目盛り数字のサイズ
plt.rcParams['legend.fontsize'] = 9              # 凡例の文字サイズ

# ==============================================================================
# 3. グラフの作成（幅15cm）
# ==============================================================================
fig_width_in = 15 / 2.54  # 幅15cmをインチに変換
fig_height_in = 2.5     # 横に3つ並べたときにバランスの良い高さ

fig, axes = plt.subplots(1, 3, figsize=(fig_width_in, fig_height_in))

# グラフの設定用データ構造
plot_configs = [
    {"title": "Water",     "ol": water_ol, "pid": water_pid, "str": water_str},
    {"title": "Miso Soup", "ol": miso_ol,  "pid": miso_pid,  "str": miso_str},
    {"title": "Miso Soup +Tofu", "ol": tofu_ol,  "pid": tofu_pid,  "str": tofu_str}
]

colors = {'OL': '#4d4d4d', 'PID': '#1f77b4', 'STR': "#ce0000"}

for i, cfg in enumerate(plot_configs):
    ax = axes[i]
    
    # 3つの制御モードをプロット
    ax.plot(steps, cfg["ol"],  label='Open-Loop', color=colors['OL'], alpha=0.6, linewidth=1.5)
    ax.plot(steps, cfg["pid"], label='PID',       color=colors['PID'], alpha=0.8, linewidth=1.5)
    ax.plot(steps, cfg["str"], label='STR',       color=colors['STR'], linewidth=2.0)
    
    # 目標ライン（1.0% の黒点線、半透明）
    ax.axhline(1.0, color='black', linestyle=':', alpha=0.5, label='Target (1.0%)')
    
    # グラフの装飾
    ax.set_title(cfg["title"], fontweight='bold')
    ax.set_xlabel("Step (30 s/step)")
    ax.set_xlim(1, 24)
    ax.set_xticks([1, 6, 12, 18, 24])
    
    # Y軸ラベルと凡例は一番左のグラフ（Water）のみに表示してすっきりさせる
    if i == 0:
        ax.set_ylabel("Real Salinity [%]")
        
    if i == 2:                       # 💡 一番右のグラフ（Miso+Tofu）のとき
        ax.legend(loc='lower right') # 💡 凡例を右下に表示
        
    ax.set_ylim(0, 1.7)     # Y軸の範囲（0〜1.5%）
    ax.grid(alpha=0.3)

# レイアウトの自動調整（文字の見切れないようにする）
plt.tight_layout()

plt.savefig("Fig9.png", dpi=300, bbox_inches='tight')
plt.savefig("Fig9.pdf", bbox_inches='tight')

plt.show()


