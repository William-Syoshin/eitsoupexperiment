import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. データの準備
# ==========================================
# 💡 修正1：列名に改行（\n）を入れて、横幅を節約する
data = {
    "Soup Type": ["Pure Water", "Miso Soup", "Miso Soup\n+Tofu"],
    "Water\n(g)": [600, 580, 480],
    "Miso\n(g)": [0, 20, 20],
    "Tofu\n(g)": [0, 0, 100],
    "Total\n(g)": [600, 600, 600]
}
df = pd.DataFrame(data)

# ==========================================
# 2. 画像フォーマットの設定 (幅8cm)
# ==========================================
fig_width_inch = 8.0 / 2.54
fig_height_inch = 4.0 / 2.54  # 改行に合わせて高さを少し広げました

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['font.size'] = 10  # 💡 修正2：フォントサイズを10にアップ

fig, ax = plt.subplots(figsize=(fig_width_inch, fig_height_inch))
ax.axis('off')

# ==========================================
# 3. 表（Table）の描画
# ==========================================
# 💡 修正3：一番左の幅を削り（36%→28%）、残りの列を広く（16%→18%）する
col_widths = [0.28, 0.18, 0.18, 0.18, 0.18]

table = ax.table(cellText=df.values, 
                 colLabels=df.columns, 
                 loc='center', 
                 cellLoc='center',
                 colWidths=col_widths)

table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1.0, 2.2)  # 💡 修正4：改行が入る分、セルの縦幅(高さ)に余裕を持たせる

# ==========================================
# 罫線の装飾（Booktabs スタイル）
# ==========================================
for (row, col), cell in table.get_celld().items():
    cell.set_linewidth(0)
    
    if row == 0:
        cell.set_text_props(weight='bold')
        cell.visible_edges = 'BT'
        cell.set_edgecolor('black')
        cell.set_linewidth(1.0)
        
    elif row == len(df):
        cell.visible_edges = 'B'
        cell.set_edgecolor('black')
        cell.set_linewidth(1.0)
        
    else:
        cell.visible_edges = 'B'
        cell.set_edgecolor('#cccccc')
        cell.set_linewidth(0.5)

plt.tight_layout()

# ==========================================
# 4. 画像の保存
# ==========================================
plt.savefig('Table1.png', dpi=600, bbox_inches='tight')
plt.savefig('Table1.pdf', bbox_inches='tight')

print("表の画像を保存しました (Table1.png / pdf)")
plt.show()