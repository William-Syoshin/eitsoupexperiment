# -*- coding: utf-8 -*-
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. データの準備
# ==========================================
# 💡 「Miso」の列に塩分濃度8.9%を追記し、初期塩分の数値を小数点以下3桁で更新しました
data = {
    "Soup Type": ["Pure Water", "Miso Soup", "Miso Soup\n+Tofu"],
    "Water\n(g)": [600, 580, 480],
    "Miso\n(g)": [0, 20, 20],
    "Tofu\n(g)": [0, 0, 100],
    "Total\n(g)": [600, 600, 600],
    "Init. Salt\n(%)": ["0.000", "0.288", "0.344"]
}
df = pd.DataFrame(data)

# ==========================================
# 2. 画像フォーマットの設定 (幅8cm)
# ==========================================
fig_width_inch = 8.0 / 2.54
fig_height_inch = 4.2 / 2.54  

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['font.size'] = 9  

fig, ax = plt.subplots(figsize=(fig_width_inch, fig_height_inch))
ax.axis('off')

# ==========================================
# 3. 表（Table）の描画
# ==========================================
# 💡 Misoの列名が少し長くなったため、幅の比率を微調整して綺麗に収まるようにしました
col_widths = [0.24, 0.14, 0.18, 0.14, 0.14, 0.16]

table = ax.table(cellText=df.values, 
                 colLabels=df.columns, 
                 loc='center', 
                 cellLoc='center',
                 colWidths=col_widths)

table.auto_set_font_size(False)
table.set_fontsize(9)  
table.scale(1.0, 2.2)

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
filename_png = 'Table1_Conditions.png'
filename_pdf = 'Table1_Conditions.pdf'

plt.savefig(filename_png, dpi=600, bbox_inches='tight')
plt.savefig(filename_pdf, bbox_inches='tight')

print(f"表の画像を保存しました: {filename_png} / {filename_pdf}")
plt.show()