import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import io

# ==========================================
# 1. データの入力エリア（味噌汁環境のデータを設定済み）
# ==========================================
data_text = """
試行	C (g)	T (°C)	σ (mS/cm)
1	0	54.28	6.42
1	1	54.5	8.01
1	2	54.14	11.02
1	3	52.82	15.35
1	4	51.55	18.41
1	5	50.76	21.6
1	6	49.77	23.3
1	7	54.17	24.9
1	8	52.84	26.8
2	0	54.07	5.8
2	1	52.7	8.08
2	2	51.7	11.26
2	3	49.44	14.66
2	4	51.65	17.86
2	5	50.08	20.8
2	6	51.72	22.3
2	7	51	23.4
2	8	49.57	24
3	0	53.76	6.88
3	1	52.47	8.15
3	2	50.42	10.04
3	3	51.14	12.44
3	4	51.99	15.76
3	5	52.37	18.95
3	6	51.51	21.3
3	7	48.51	22.4
3	8	50.32	25.4
"""

# ==========================================
# 2. 定数の設定
# ==========================================
T_room = 24.6          # 室温 (°C)
a_temp = 0.02          # 温度係数
M_water = 480.0        # 水の質量 (g)
M_miso = 20.0          # 味噌の質量 (g)
# 豆腐100gは固形物（非溶媒）として計算から完全に除外します

# 味噌由来の初期塩分を計算 (20g * 8.7%)
miso_salt_ratio = 0.087
initial_salt = M_miso * miso_salt_ratio  # = 1.74g

# ベースとなる液相の質量（塩を入れる前）
M_base_liquid = M_water + M_miso  # = 500g

# データをデータフレームとして読み込み
df = pd.read_csv(io.StringIO(data_text.strip()), sep='\t')

# ==========================================
# 3. 計算処理 (厳密な質量パーセント濃度)
# ==========================================
# 分子: 加えた塩 C(g) + 味噌の初期塩分(1.74g)
# 分母: 水(480g) + 味噌(20g) + 加えた塩 C(g)
df['Salinity (%)'] = ((df['C (g)'] + initial_salt) / (M_base_liquid + df['C (g)'])) * 100

# 温度補正済み導電率 σ_comp の計算
df['σ_comp'] = df['σ (mS/cm)'] / (1 + a_temp * (df['T (°C)'] - T_room))

# ==========================================
# 4. 線形回帰 (最小二乗法) で a と b を求める
# ==========================================
X = df['Salinity (%)'].values
Y = df['σ_comp'].values

slope, intercept, r_value, p_value, std_err = linregress(X, Y)

a = slope
b = intercept
r_squared = r_value**2

print("=== Env 3 (厳密な濃度定義) フィッティング結果 ===")
print(f"ベース液相質量: {M_base_liquid} g (水480g + 味噌20g)")
print(f"味噌由来の初期塩分: {initial_salt:.2f} g")
print(f"モデル式: σ_comp = a * C + b")
print(f"a (感度 α_miso_tofu): {a:.4f}")
print(f"b (絶対ベースライン β_miso_tofu): {b:.4f}")
print(f"決定係数 R^2:    {r_squared:.4f}")

# ==========================================
# 5. グラフの描画
# ==========================================
plt.figure(figsize=(8, 6))

# 試行ごとに色を分けてプロット
colors = {1: 'blue', 2: 'green', 3: 'red'}
for trial in df['試行'].unique():
    subset = df[df['試行'] == trial]
    plt.scatter(subset['Salinity (%)'], subset['σ_comp'], 
                label=f'Trial {trial}', color=colors.get(trial, 'black'))

# フィッティング直線のプロット
x_line = np.linspace(X.min(), X.max(), 100)
y_line = a * x_line + b
plt.plot(x_line, y_line, color='orange', linewidth=2, label=f'Fit: y={a:.2f}x + {b:.2f}')

plt.xlabel('Strict Salinity Concentration [%]\n((Added Salt + 1.74g) / (500g + Added Salt))')
plt.ylabel('Compensated Conductivity σ_comp [mS/cm]')
plt.title('Calibration Curve for Miso Soup + Tofu (Strict Definition)')
plt.legend()
plt.grid(True)
plt.show()