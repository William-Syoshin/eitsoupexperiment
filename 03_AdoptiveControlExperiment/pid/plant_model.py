"""
pid.py
======
温度・塩分濃度の 2ループ PID 制御シミュレーション

【制御構造】案A：温度補償アプローチ
  ─────────────────────────────────────────────────────────
  ループ①（温度）
    T_ref ─→ PID① ─→ Q_in ─→ [プラント] ─→ T_w ─→ フィードバック

  ループ②（濃度）
    C_ref ─→ PID② ─→ salt_rate ─→ [プラント] ─→ σ
                                                    ↓
                                           [温度補償] f⁻¹(σ, T_w)
                                                    ↓
                                                    Ĉ ─→ フィードバック
  ─────────────────────────────────────────────────────────

【1ステップ内の処理順序（重要）】
  1. 温度PID  → Q_in を計算
  2. プラントを熱のみ1ステップ進める（salt_added=0）→ T_w 更新
  3. 更新された T_w で σ→Ĉ 変換（温度補償）
  4. 濃度PID  → salt_rate を計算 → salt_added [g] に変換
  5. 塩をプラントに追加（熱ダイナミクスは動かさない）

  ※ ステップ2の後に T_w を使うことで、
    「温度PIDの結果を反映した T_w」で濃度を推定できる。
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from soup_plant import SoupPlant

# 日本語フォント設定
_jp_fonts = ['Hiragino Sans', 'Yu Gothic', 'Meiryo', 'Noto Sans CJK JP', 'IPAexGothic']
_available = {f.name for f in fm.fontManager.ttflist}
for _f in _jp_fonts:
    if _f in _available:
        plt.rcParams['font.family'] = _f
        break
plt.rcParams['axes.unicode_minus'] = False


# ============================================================
# PIDコントローラ
# ============================================================

class PIDController:
    """
    汎用PIDコントローラ（アンチワインドアップ付き）

    アンチワインドアップとは:
        出力が飽和（上限/下限に張り付いている）している間は
        積分を蓄積しないようにする仕組み。
        これがないと飽和中に積分が膨らみ続け、
        飽和が解けた後にオーバーシュートが大きくなる。
    """

    def __init__(self,
                 Kp: float,
                 Ki: float,
                 Kd: float,
                 output_min: float = None,
                 output_max: float = None):
        """
        Parameters
        ----------
        Kp, Ki, Kd  : PIDゲイン
        output_min  : 出力下限（例: Q_in >= 0, salt_rate >= 0）
        output_max  : 出力上限（None = 制限なし）
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral   = 0.0   # 積分項の累積値
        self._prev_error = None  # 前ステップの偏差（微分用）

    def compute(self, error: float, dt: float) -> float:
        """
        偏差から PID 出力を計算する

        Parameters
        ----------
        error : 偏差 = setpoint - measurement
        dt    : 時間刻み [s]

        Returns
        -------
        output : PID出力（飽和処理・アンチワインドアップ済み）
        """
        # ── P 項（比例）──────────────────────────────────────
        # 今の偏差に即座に反応する。Kp が大きいほど応答が速い。
        P = self.Kp * error

        # ── I 項（積分）──────────────────────────────────────
        # 偏差を時間積分して定常偏差（オフセット）をなくす。
        # Ki が大きいほど速く定常偏差を消せるが、振動しやすくなる。
        self._integral += error * dt
        I = self.Ki * self._integral

        # ── D 項（微分）──────────────────────────────────────
        # 偏差の変化速度に反応してブレーキをかける。
        # 初回ステップは前回値がないので 0 とする。
        if self._prev_error is None:
            D = 0.0
        else:
            D = self.Kd * (error - self._prev_error) / dt
        self._prev_error = error

        # ── PID 出力の合計 ────────────────────────────────────
        output = P + I + D

        # ── 飽和処理 ＋ アンチワインドアップ ─────────────────
        # 出力が上限/下限を超えたら、今回蓄積した積分を巻き戻す。
        # （= 飽和中は積分を止めるのと同じ効果）
        if self.output_max is not None and output > self.output_max:
            self._integral -= error * dt   # 今回分の積分を取り消す
            output = self.output_max

        if self.output_min is not None and output < self.output_min:
            self._integral -= error * dt   # 今回分の積分を取り消す
            output = self.output_min

        return output

    def reset(self):
        """積分と前回偏差をリセット（シミュレーション再実行時に使用）"""
        self._integral   = 0.0
        self._prev_error = None


# ============================================================
# メインシミュレーション
# ============================================================

if __name__ == "__main__":

    # ── シミュレーション設定 ─────────────────────────────────
    DT       = 1.0    # 時間刻み [s]
    SIM_TIME = 600.0  # シミュレーション時間 [s]（10分間）
    steps    = int(SIM_TIME / DT)
    time     = np.arange(steps) * DT

    # ── 目標値 ───────────────────────────────────────────────
    T_REF = 50.0   # 目標温度 [°C]
    C_REF = 1.0    # 目標塩分濃度 [%]（増やす方向のみ）

    # ── プラント生成 ─────────────────────────────────────────
    plant = SoupPlant(
        water_mass=500,
        potato_mass=100,
        T_w_init=20.0,   # 初期水温
        T_p_init=20.0,   # 初期芋温
        salt_init=0.0,   # 塩なし（初期状態）
    )

    # ── PID① 温度制御 ────────────────────────────────────────
    # 操作量: Q_in [°C/s]
    # 制約: Q_in >= 0（IHは冷やせない）、上限なし
    # ★ゲインは実験で調整が必要。以下は初期値。
    pid_temp = PIDController(
        Kp=0.05,          # 比例ゲイン: 大きくすると応答速い→振動注意
        Ki=0.0005,        # 積分ゲイン: 定常偏差を消す
        Kd=2.0,           # 微分ゲイン: オーバーシュートを抑える
        output_min=0.0,   # IHは冷却不可 → 0以下はカット
        output_max=None,  # 上限なし（指定があれば設定）
    )

    # ── PID② 濃度制御 ────────────────────────────────────────
    # 操作量: salt_rate [g/s]（1ステップの添加速度）
    # 制約: salt_rate >= 0（塩は抜けない）
    # ★オーバーシュートOKの設定。Kiを大きめにして速く追従させる。
    pid_conc = PIDController(
        Kp=0.5,           # 比例ゲイン
        Ki=0.005,         # 積分ゲイン
        Kd=0.0,           # 微分ゲイン: 濃度変化は緩やかなので0でOK
        output_min=0.0,   # 塩は減らせない → 0以下はカット
        output_max=None,
    )

    # ── ログ用配列（結果を記録） ─────────────────────────────
    log_Tw        = np.zeros(steps)   # 水温
    log_Tp        = np.zeros(steps)   # 芋温
    log_C         = np.zeros(steps)   # 塩分濃度（真値）
    log_C_hat     = np.zeros(steps)   # 推定濃度 Ĉ（温度補償後）
    log_sigma     = np.zeros(steps)   # 電気伝導率 σ
    log_Q_in      = np.zeros(steps)   # 操作量: 加熱パワー
    log_salt_step = np.zeros(steps)   # 1ステップごとの塩添加量 [g]
    log_salt_cum  = np.zeros(steps)   # 累積塩添加量 [g]

    # ============================================================
    # シミュレーションループ
    # ============================================================
    for i in range(steps):

        # ── ステップ1: 温度PID ───────────────────────────────
        # 偏差 = 目標温度 - 現在の水温
        e_T  = T_REF - plant.T_w
        Q_in = pid_temp.compute(e_T, DT)
        # output_min=0 で既に処理されているが明示的にも保証
        Q_in = max(0.0, Q_in)

        # ── ステップ2: プラントを熱のみ1ステップ進める ─────────
        # salt_added=0 で熱ダイナミクスだけを更新する。
        # この結果として T_w が更新される。
        plant.step(Q_in=Q_in, salt_added=0.0, dt=DT)

        # ── ステップ3: 温度補償 σ → Ĉ 変換 ─────────────────
        # 「更新後の T_w」を使って σ を真の濃度 Ĉ に逆変換する。
        # Ĉ = ( σ / (1 + a*(T_w - T_base)) - β ) / α
        # ※ estimated_concentration は soup_plant.py に実装済み
        C_hat = plant.estimated_concentration

        # ── ステップ4: 濃度PID ───────────────────────────────
        # 偏差 = 目標濃度 - 推定濃度（温度補償済み）
        e_C       = C_REF - C_hat
        salt_rate = pid_conc.compute(e_C, DT)   # [g/s]
        # 1ステップ分の添加量 [g] に変換（負の添加は不可）
        salt_added = max(0.0, salt_rate) * DT

        # ── ステップ5: 塩をプラントに追加 ────────────────────
        # 熱ダイナミクスは動かさず塩だけ加える。
        # soup_plant.py の step() を使わず直接更新することで
        # 「このタイムステップ内でT_wを2回変化させない」を保証。
        plant.salt_mass   += salt_added
        plant.liquid_mass += salt_added

        # ── ログ記録 ─────────────────────────────────────────
        log_Tw       [i] = plant.T_w
        log_Tp       [i] = plant.T_p
        log_C        [i] = plant.concentration
        log_C_hat    [i] = plant.estimated_concentration
        log_sigma    [i] = plant.conductivity
        log_Q_in     [i] = Q_in
        log_salt_step[i] = salt_added        # このステップで加えた塩 [g]
        log_salt_cum [i] = plant.salt_mass   # 累積投入量 [g]

    # ============================================================
    # 結果サマリー
    # ============================================================
    print("=" * 55)
    print("  2ループ PID 制御シミュレーション 結果")
    print("=" * 55)
    print(f"  目標温度  T_ref = {T_REF}°C")
    print(f"  → 最終 T_w     = {plant.T_w:.2f}°C")
    print()
    print(f"  目標濃度  C_ref = {C_REF}%")
    print(f"  → 最終 Ĉ      = {plant.estimated_concentration:.3f}%")
    print(f"  → 最終 C（真値）= {plant.concentration:.3f}%")
    print()
    print(f"  最終 σ         = {plant.conductivity:.3f} mS/cm")
    print(f"  投入塩 合計     = {plant.salt_mass:.2f} g")
    print("=" * 55)

    # ============================================================
    # グラフ描画（3行 × 2列 = 6枠、5枚使用）
    # ============================================================
    fig, axes = plt.subplots(3, 2, figsize=(13, 12))
    fig.suptitle("2-Loop PID Control: Temperature + Salt Concentration (Case A: Temp. Compensation)",
                 fontsize=13, fontweight="bold")

    # ① Temperature profile
    ax = axes[0, 0]
    ax.plot(time, log_Tw, color="#C0392B", lw=2.2, label="T_w (water temp.)")
    ax.plot(time, log_Tp, color="#E67E22", lw=1.8, ls="--", label="T_p (potato temp.)")
    ax.axhline(T_REF, color="gray", ls=":", lw=1.5, label=f"T_ref = {T_REF} °C")
    ax.set(xlabel="Time [s]", ylabel="Temperature [°C]", title="① Temperature")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    # ② Heating power Q_in (manipulated variable ①)
    ax = axes[0, 1]
    ax.plot(time, log_Q_in, color="#8E44AD", lw=2.0)
    ax.set(xlabel="Time [s]", ylabel="Q_in [°C/s]", title="② Manipulated Variable: Heating Power Q_in")
    ax.grid(alpha=0.3)

    # ③ Salt concentration (true value vs. estimated Ĉ)
    ax = axes[1, 0]
    ax.plot(time, log_C,     color="#27AE60", lw=2.2, label="C (true value)")
    ax.plot(time, log_C_hat, color="#1ABC9C", lw=1.8, ls="--", label="C_hat (temp.-compensated estimate)")
    ax.axhline(C_REF, color="gray", ls=":", lw=1.5, label=f"C_ref = {C_REF} %")
    ax.set(xlabel="Time [s]", ylabel="Salt Concentration [%]", title="③ Salt Concentration (True vs. Estimated)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    # ④ Electrical conductivity σ (sensor output)
    ax = axes[1, 1]
    ax.plot(time, log_sigma, color="#2980B9", lw=2.0, label="sigma (sensor output)")
    ax.set(xlabel="Time [s]", ylabel="sigma [mS/cm]", title="④ Electrical Conductivity sigma")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    # ⑤ Salt added (per step + cumulative)
    ax = axes[2, 0]
    ax2 = ax.twinx()   # cumulative on right axis
    ax.bar(time, log_salt_step, width=DT, color="#D35400", alpha=0.6, label="Added per step [g]")
    ax2.plot(time, log_salt_cum, color="#922B21", lw=2.0, label="Cumulative total [g]")
    ax.set(xlabel="Time [s]", ylabel="Salt added [g/step]", title="⑤ Salt Input")
    ax2.set_ylabel("Cumulative salt [g]")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=9)
    ax.grid(alpha=0.3)

    # ⑥ Reserved for future use
    axes[2, 1].axis("off")

    plt.tight_layout()
    plt.savefig("pid_simulation.png", dpi=150, bbox_inches="tight")
    print("\nグラフ保存: pid_simulation.png")
    plt.close()