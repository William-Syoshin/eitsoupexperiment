import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# ==========================================
# 1. 実験データの入力（ここにエクセルの値を直接書く）
# ==========================================
# 塩の量 (g)
S_data = np.array([0, 1, 2, 3, 4, 5, 6])

# 水だけの伝導率 (ms/cm) 
# ※以下の数字を、エクセルの青線の正確な値に書き換えてください！
sigma_data = np.array([0.259, 3.64, 5.34, 7.39, 9.43, 11.6, 13.9]) 

# データ測定時の温度（50度）
T_exp = 50.0

# ==========================================
# 2. モデル式の定義
# ==========================================
# σ = (aS + B) * (1 + 0.02(T - 25))
def conductivity_model(S, a, B):
    # 温度補正係数 (50度なら 1 + 0.02*25 = 1.5)
    temp_correction = 1.0 + 0.02 * (T_exp - 25.0) 
    return (a * S + B) * temp_correction

# ==========================================
# 3. フィッティングの実行
# ==========================================
# curve_fitが、データに最も合う a と B を自動計算してくれます
popt, pcov = curve_fit(conductivity_model, S_data, sigma_data)
a_fit = popt[0]
B_fit = popt[1]

#★追加：R^2 (決定係数) の計算
# ① まず、求まった a, B を使って「理論上の伝導率」を計算する
sigma_pred = conductivity_model(S_data, a_fit, B_fit)

# ② 実際のデータとのズレ（残差平方和）と、データ全体のばらつき（全平方和）から R^2 を出す
ss_res = np.sum((sigma_data - sigma_pred) ** 2)
ss_tot = np.sum((sigma_data - np.mean(sigma_data)) ** 2)
r_squared = 1 - (ss_res / ss_tot)
print("========================================")
print(f"求まったパラメータ:")
print(f"a (傾き) = {a_fit:.4f}")
print(f"B (切片) = {B_fit:.4f}")
print("========================================")

# ==========================================
# 4. グラフで確認（ズレがないか視覚的にチェック）
# ==========================================
# 綺麗な線を引くための計算
S_line = np.linspace(0, 6, 100)
sigma_line = conductivity_model(S_line, a_fit, B_fit)

plt.figure(figsize=(8, 6))
# 実際のデータを点でプロット
plt.scatter(S_data, sigma_data, color='blue', s=50, label='Excel Data (Water)')

# ★追加：グラフの凡例（ラベル）にも R^2 を表示するようにしました
plt.plot(S_line, sigma_line, color='red', 
         label=f'Fitted Model: a={a_fit:.3f}, B={B_fit:.3f}\n$R^2$={r_squared:.4f}')
plt.title('Conductivity Fitting: $\sigma = (aS + B)(1 + 0.02(T - 25))$')
plt.xlabel('Salt (g)')
plt.ylabel('Conductivity (ms/cm)')
plt.legend()
plt.grid(True)
plt.show()