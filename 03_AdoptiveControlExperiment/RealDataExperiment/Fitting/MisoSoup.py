import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import io

# ==========================================
# 1. データの入力エリア（水580g + 味噌20gのデータを設定済み）
# ==========================================
data_text = """
試行	C (g)	T (°C)	σ (mS/cm)
1	0	51.99	6.16
1	1	51.62	8.38
1	2	50.35	10.8
1	3	49.49	13.33
1	4	50.56	16.35
1	5	51.27	19.15
1	6	50.42	21.7
1	7	49.43	22.3
1	8	52.00	24.5
2	0	53.70	5.53
2	1	51.92	6.93
2	2	50.42	8.75
2	3	51.51	11.71
2	4	51.85	13.99
2	5	48.45	17.4
2	6	46.74	19.01
2	7	49.16	21.0
2	8	52.47	21.9
3	0	51.21	6.13
3	1	50.12	8.38
3	2	48.56	10.66
3	3	51.85	13.09
3	4	50.76	15.79
3	5	49.26	18.23
3	6	51.82	20.0
3	7	49.91	20.9
3	8	48.50	22.6
"""

# ==========================================
# 2. 定数の設定
# ==========================================
T_room = 24.6      # 室温 (°C)
a_temp = 0.02      # 温度係数
M_water = 580.0    # 水の質量 (g) ← 580gに変更！

# データをデータフレームとして読み込み
df = pd.read_csv(io.StringIO(data_text.strip()), sep='\t')

# ==========================================
# 3. 計算処理
# ==========================================
# 塩分濃度 C(%) の計算: (塩の質量 / 水の質量(580g)) * 100
df['Salinity (%)'] = (df['C (g)'] / M_water) * 100

# 温度補正済み導電率 σ_comp の計算
# σ_comp = σ / (1 + 0.02 * (T - T_room))
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

print("=== 水580g + 味噌20g フィッティング結果 ===")
print(f"モデル式: σ_comp = a * C + b")
print(f"a (感度): {a:.4f}")
print(f"b (初期ベースライン): {b:.4f}")
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

plt.xlabel('Salinity C(t) [%] (Base: 580g Water)')
plt.ylabel('Compensated Conductivity σ_comp [mS/cm]')
plt.title('Calibration Curve for Water (580g) + Miso (20g)')
plt.legend()
plt.grid(True)
plt.show()