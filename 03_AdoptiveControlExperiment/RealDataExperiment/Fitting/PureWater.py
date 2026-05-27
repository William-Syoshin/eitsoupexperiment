import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import io

# ==========================================
# 1. データの入力エリア（ここに手打ち・コピペしてください）
# ==========================================
data_text = """
試行	C (g)	T (°C)	σ (mS/cm)
1	0	50.73	0.242
1	1	53.05	2.160
1	2	52.26	4.610
1	3	51.31	6.210
1	4	50.66	7.940
1	5	49.98	10.12
1	6	51.21	12.00
1	7	53.45	13.68
1	8	53.22	15.14
2	0	49.33	0.259
2	1	48.45	2.030
2	2	49.98	4.160
2	3	50.59	5.610
2	4	51.62	7.200
2	5	49.23	8.730
2	6	46.44	10.21
2	8	51.62	11.51
2	9	53.59	12.80
3	0	49.33	0.218
3	1	50.79	2.820
3	2	50.32	5.500
3	3	49.74	7.120
3	4	51.62	9.220
3	5	51.62	11.20
3	6	50.35	12.61
3	7	49.35	13.77
3	8	48.55	16.55
"""

# ==========================================
# 2. 定数の設定
# ==========================================
T_room = 24.6      # 室温 (°C)
a_temp = 0.02      # 温度係数
M_water = 600.0    # 水の質量 (g)

# データをデータフレームとして読み込み
df = pd.read_csv(io.StringIO(data_text.strip()), sep='\t')

# ==========================================
# 3. 計算処理
# ==========================================
# 塩分濃度 C(%) の計算: (塩の質量 / 水の質量) * 100
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

print("=== フィッティング結果 ===")
print(f"モデル式: σ_comp = a * C + b")
print(f"a (感度 α_water): {a:.4f}")
print(f"b (切片 β_water): {b:.4f}")
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

plt.xlabel('Salinity C(t) [%]')
plt.ylabel('Compensated Conductivity σ_comp [mS/cm]')
plt.title('Calibration Curve for Pure Water (All Trials Pooled)')
plt.legend()
plt.grid(True)
plt.show()