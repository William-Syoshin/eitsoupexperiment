import numpy as np
import matplotlib.pyplot as plt

# --- 各スープの真の物理パラメータ ---
# 導電率の式: sigma = alpha * C + beta
soups = {
    "Water (PID's Belief)": {"alpha": 11.9320, "beta": 0.3410,  "color": "blue",   "ls": "--"},
    "Potato Soup":          {"alpha": 8.5000,  "beta": 1.5000,  "color": "green",  "ls": "-"},
    "Miso Tofu Soup":       {"alpha": 10.8500, "beta": 5.1200,  "color": "orange", "ls": "-"}
}

# 塩分濃度 0.0% 〜 1.5% の範囲
concentrations = np.linspace(0, 1.5, 100)

plt.figure(figsize=(10, 6))

# 各スープの直線をプロット
for name, params in soups.items():
    sigma = params["alpha"] * concentrations + params["beta"]
    plt.plot(concentrations, sigma, label=f'{name} ($\\alpha$={params["alpha"]}, $\\beta$={params["beta"]})',
             color=params["color"], linestyle=params["ls"], linewidth=2.5)

# --- ターゲット（1.0%）のライン ---
target_C = 1.0
plt.axvline(target_C, color='black', linestyle=':', linewidth=1.5, label='Target Concentration (1.0%)')

# 水（PIDの基準）が1.0%のときの伝導率を計算して水平線を引く
water_sigma_at_target = soups["Water (PID's Belief)"]["alpha"] * target_C + soups["Water (PID's Belief)"]["beta"]
plt.axhline(water_sigma_at_target, color='gray', linestyle=':', linewidth=1.5)

# グラフの装飾
plt.title("Why PID Fails: Electrical Conductivity vs. Salinity Concentration", fontsize=14, fontweight="bold")
plt.xlabel("Salinity Concentration [%]", fontsize=12)
plt.ylabel("Electrical Conductivity [mS/cm] (at 25°C)", fontsize=12)
plt.xlim(0, 1.5)
plt.ylim(0, 25)
plt.grid(alpha=0.4)
plt.legend(fontsize=11, loc="upper left")

# テキスト注釈（PIDの勘違いポイント）
plt.annotate('PID expects 1.0% to be here', 
             xy=(1.0, water_sigma_at_target), 
             xytext=(0.5, water_sigma_at_target + 2),
             arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=6),
             fontsize=10)

plt.tight_layout()
plt.show()