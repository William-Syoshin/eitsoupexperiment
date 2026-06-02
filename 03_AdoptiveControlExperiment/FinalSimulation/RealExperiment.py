"""
RealExperiment.py
=================
実機実験用メインスクリプト。

【1サイクル（60秒）の構造】
  0 ─────────────── 50s ──────────────── 55s ──────── 60s
  [監視・ダッシュボード更新]  [EC入力＋温度5点計測（並行）]  [制御計算・塩投入]

使い方:
    1. Arduino を USB 接続し、SERIAL_PORT を設定する
    2. python3 RealExperiment.py
    3. 50秒ごとに市販センサーのEC値を入力する（温度計測と並行）
    4. Ctrl+C で終了 → グラフ表示
"""

import csv
import re
import threading
import time
import math
import unicodedata
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.console import Console
from rich import box

from Arduino_Interface import ArduinoInterface
from Controls.AdaptiveControl import STRController
from Controls.PIDcontrol import PIDController

# ─────────────────────────────────────────
# 設定
# ─────────────────────────────────────────
SERIAL_PORT        = "/dev/cu.usbserial-10"
BAUDRATE           = 115200

SALT_INTERVAL      = 60.0   # 1サイクルの長さ [s]
TEMP_COLLECT_START = 50.0   # 温度計測・EC入力開始（サイクル内の秒数）
ACTION_TRIGGER     = 55.0   # 制御計算・塩投入開始（サイクル内の秒数）
TOTAL_TIME         = 720.0  # 実験時間 [s]（12分）

C_TARGET           = 1.0    # 目標塩分濃度 [%]
M_TOTAL            = 600.0  # スープ総質量 [g]
SALT_MAX_PER_STEP  = 1.0    # 1ステップの最大投入量 [g]

# 純水キャリブレーション値（市販センサー mS/cm ベース）
ALPHA_NOMINAL = 6.63
BETA_NOMINAL  = 0.69
TEMP_COEFF    = 0.02
T_BASE        = 24.6

KP = 0.1
KI = 0.005
KD = 0.0

CONTROL_MODE = "PID"   # "STR" / "PID" / "OpenLoop"


# ─────────────────────────────────────────
# 手動EC入力
# ─────────────────────────────────────────

def get_manual_ec(console: Console, last_ec: float, T_now: float) -> float:
    console.print()
    console.rule("[bold yellow]EC 入力（50〜55s：温度計測と並行）[/bold yellow]")
    console.print(f"  現在の鍋温度: [cyan]{T_now:.2f} °C[/cyan]  |  目標: [cyan]{C_TARGET} %[/cyan]")
    console.print()

    while True:
        if math.isnan(last_ec):
            prompt = "  市販センサーのEC値を入力 [mS/cm]: "
        else:
            prompt = f"  EC値を入力 [mS/cm] (前回 {last_ec:.3f}, Enterでスキップ): "

        ec_str = console.input(f"[bold cyan]{prompt}[/bold cyan]").strip()
        # ANSIエスケープシーケンス除去（矢印キー等が混入する場合の対策）
        ec_str = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", ec_str).strip()
        # 全角数字・小数点・カンマを半角に正規化（日本語IME対策）
        ec_str = unicodedata.normalize("NFKC", ec_str).replace(",", ".")

        if ec_str == "":
            if not math.isnan(last_ec):
                console.print(f"  [dim]前回値 {last_ec:.3f} mS/cm を使用[/dim]")
                return last_ec
            console.print("  [red]初回は必ず入力してください[/red]")
            continue

        try:
            val = float(ec_str)
            if val < 0:
                console.print("  [red]正の値を入力してください[/red]")
                continue
            return val
        except ValueError:
            console.print(f"  [red]数値で入力してください（例: 2.341）  受け取った値: {repr(ec_str)}[/red]")


# ─────────────────────────────────────────
# 塩分濃度の推定
# ─────────────────────────────────────────

def estimate_concentration(sigma, T, alpha, beta):
    temp_factor = 1.0 + TEMP_COEFF * (T - T_BASE)
    if temp_factor == 0 or alpha == 0:
        return 0.0
    return (sigma / temp_factor - beta) / alpha


# ─────────────────────────────────────────
# ダッシュボード生成
# ─────────────────────────────────────────

def make_dashboard(plant, manual_ec, elapsed, total_time,
                   C_est, total_salt, control_mode, C_target,
                   last_action, alpha_h, beta_h,
                   cycle_pos, T_avg):

    layout = Layout()
    layout.split_column(
        Layout(name="top",  size=3),
        Layout(name="main"),
    )
    layout["main"].split_row(
        Layout(name="monitor", ratio=5),
        Layout(name="status",  ratio=4),
    )

    # ── タイトルバー ──
    progress = min(elapsed / total_time, 1.0)
    bar_len  = 40
    bar      = "█" * int(bar_len * progress) + "░" * (bar_len - int(bar_len * progress))
    title_text = Text()
    title_text.append(f" Adaptive Salinity Control  [{control_mode}]  ",
                      style="bold white on navy_blue")
    title_text.append(f"  {bar}  {elapsed:.0f}/{total_time:.0f} s",
                      style="cyan")
    layout["top"].update(Panel(title_text, box=box.HORIZONTALS,
                               border_style="navy_blue"))

    # ── シリアルモニタ（左パネル） ──
    lines = plant.raw_lines[-18:]
    monitor_text = Text()
    for line in lines:
        if line.startswith("ACK:"):
            monitor_text.append(line + "\n", style="bold green")
        elif line.startswith("ERR:"):
            monitor_text.append(line + "\n", style="bold red")
        elif line == "READY":
            monitor_text.append(line + "\n", style="bold yellow")
        else:
            monitor_text.append(line + "\n", style="bright_white")
    layout["monitor"].update(
        Panel(monitor_text, title="[bold green]Serial Monitor[/bold green]",
              border_style="green", box=box.ROUNDED)
    )

    # ── ステータスパネル（右パネル） ──
    tbl = Table(box=box.SIMPLE, show_header=False, padding=(0, 1))
    tbl.add_column("key",   style="cyan",       no_wrap=True)
    tbl.add_column("value", style="bold white", no_wrap=True)

    # フェーズ表示
    if cycle_pos < TEMP_COLLECT_START:
        phase = f"[green]監視中[/green]  → {TEMP_COLLECT_START - cycle_pos:.0f}s後に計測・入力"
    elif cycle_pos < ACTION_TRIGGER:
        phase = "[yellow]EC入力 ＆ 温度計測中[/yellow]"
    else:
        phase = "[red]制御計算・塩投入中[/red]"

    t_col = ("red"    if plant.T_w > 55
             else "yellow" if plant.T_w > 45
             else "white")
    c_col = ("green"  if abs(C_est - C_target) < 0.05
             else "yellow" if abs(C_est - C_target) < 0.2
             else "red")
    ec_str  = f"{manual_ec:.4f} mS/cm" if not math.isnan(manual_ec) else "[dim]未入力[/dim]"
    avg_str = f"{T_avg:.2f} °C"         if not math.isnan(T_avg)    else "[dim]—[/dim]"

    tbl.add_row("フェーズ",        phase)
    tbl.add_row("サイクル内",      f"{cycle_pos:.0f} / {SALT_INTERVAL:.0f} s")
    tbl.add_row("─" * 12,         "─" * 12)
    tbl.add_row("鍋温度 (現在)",   f"[{t_col}]{plant.T_w:.2f} °C[/{t_col}]")
    tbl.add_row("鍋温度 (平均)",   avg_str)
    tbl.add_row("室温",            f"{plant.T_room:.2f} °C")
    tbl.add_row("EC (手動入力)",   ec_str)
    tbl.add_row("─" * 12,         "─" * 12)
    tbl.add_row("推定塩分濃度",    f"[{c_col}]{C_est:.3f} %[/{c_col}]")
    tbl.add_row("目標濃度",        f"[bold]{C_target:.1f} %[/bold]")
    tbl.add_row("誤差",            f"[{c_col}]{C_est - C_target:+.3f} %[/{c_col}]")
    tbl.add_row("─" * 12,         "─" * 12)
    tbl.add_row("投入塩 累積",     f"{total_salt:.2f} g")
    tbl.add_row("前回の投入",      last_action)
    tbl.add_row("─" * 12,         "─" * 12)
    tbl.add_row("α̂",              f"{alpha_h:.4f}")
    tbl.add_row("β̂",              f"{beta_h:.4f}")

    layout["status"].update(
        Panel(tbl, title="[bold blue]Control Status[/bold blue]",
              border_style="blue", box=box.ROUNDED)
    )
    return layout


# ─────────────────────────────────────────
# メインループ
# ─────────────────────────────────────────

def main():
    console = Console()
    console.print(f"\n[bold]ESP32 に接続中: {SERIAL_PORT}[/bold]")

    plant = ArduinoInterface(port=SERIAL_PORT, baudrate=BAUDRATE)
    plant.connect()
    time.sleep(3.0)

    str_unit = STRController(
        alpha_init=ALPHA_NOMINAL, beta_init=BETA_NOMINAL,
        a=TEMP_COEFF, T_base=T_BASE, lam=0.98
    )
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD,
                        output_min=0.0,
                        output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)

    # ログ
    times, log_sigma, log_C, log_salt_cum = [], [], [], []
    log_alpha, log_beta, log_T_avg, log_salt_step = [], [], [], []

    # 状態変数
    total_salt        = 0.0
    C_est             = 0.0
    manual_ec         = float("nan")
    last_action       = "—"
    alpha_h           = ALPHA_NOMINAL
    beta_h            = BETA_NOMINAL
    T_avg             = float("nan")

    # サイクル管理
    last_input_cycle  = -1   # EC入力・温度計測を開始したサイクル番号
    last_action_cycle = -1   # アクション（塩投入）を実行したサイクル番号

    start_time = time.time()

    try:
        with Live(console=console, refresh_per_second=2, screen=True) as live:
            while True:
                now     = time.time()
                elapsed = now - start_time

                if elapsed >= TOTAL_TIME:
                    break

                cycle_num = int(elapsed / SALT_INTERVAL)
                cycle_pos = elapsed % SALT_INTERVAL

                # ── フェーズ1: EC入力＋温度計測（50〜55s、サイクルに1回） ──
                # EC入力（メインスレッド・ブロッキング）と
                # 温度5点計測（バックグラウンドスレッド）を並行実行する
                if (TEMP_COLLECT_START <= cycle_pos < ACTION_TRIGGER
                        and cycle_num > last_input_cycle
                        and elapsed > TEMP_COLLECT_START):

                    last_input_cycle = cycle_num

                    # バックグラウンドで5点の温度を収集（1秒おきに5回）
                    raw_temps: list[float] = []
                    temp_done = threading.Event()

                    def _collect_temps(buf=raw_temps, ev=temp_done):
                        for _ in range(5):
                            buf.append(plant.T_w)
                            time.sleep(1.0)
                        ev.set()

                    threading.Thread(target=_collect_temps, daemon=True).start()

                    # メインスレッド: EC値を手動入力（ブロッキング）
                    live.stop()
                    manual_ec = get_manual_ec(console, manual_ec, plant.T_w)

                    # 温度収集スレッドの完了を待機（最大6s）
                    temp_done.wait(timeout=6.0)
                    T_avg = float(np.mean(raw_temps)) if raw_temps else plant.T_w
                    console.print(
                        f"  [dim]温度平均: {T_avg:.2f} °C  ({len(raw_temps)} 点)[/dim]"
                    )
                    live.start()

                # ── フェーズ2: 制御計算・塩投入（55s〜、サイクルに1回） ──
                if (cycle_pos >= ACTION_TRIGGER
                        and cycle_num > last_action_cycle
                        and elapsed > ACTION_TRIGGER
                        and not math.isnan(manual_ec)):

                    last_action_cycle = cycle_num
                    sigma = manual_ec

                    # 制御計算
                    salt_g = 0.0
                    if CONTROL_MODE == "OpenLoop":
                        salt_g = (C_TARGET / 100.0) * M_TOTAL if total_salt == 0 else 0.0

                    elif CONTROL_MODE == "PID":
                        C_hat  = estimate_concentration(sigma, T_avg,
                                                        ALPHA_NOMINAL, BETA_NOMINAL)
                        error  = C_TARGET - C_hat
                        rate   = pid.compute(error, SALT_INTERVAL)
                        salt_g = rate * SALT_INTERVAL

# Wrong STR 
                    #elif CONTROL_MODE == "STR":
                        #C_hat_rls       = total_salt / M_TOTAL * 100.0
                        #alpha_h, beta_h = str_unit.estimate(sigma, C_hat_rls, T_avg)
                        #kp, ki, kd      = str_unit.get_adjusted_gains(KP, KI, KD)
                        #pid.Kp, pid.Ki, pid.Kd = kp, ki, kd
                        #C_hat_adaptive  = estimate_concentration(sigma, T_avg,
                                                                  #alpha_h, beta_h)
                        #error  = C_TARGET - C_hat_adaptive
                        #rate   = pid.compute(error, SALT_INTERVAL)
                        #salt_g = rate * SALT_INTERVAL

                    elif CONTROL_MODE == "STR":
                        # 1. ロボットの思い込み（600gの純水）ベースで、現在のパラメータを学習
                        C_hat_rls = (total_salt / M_TOTAL) * 100.0
                        alpha_h, beta_h = str_unit.estimate(sigma, C_hat_rls, T_avg)
                        
                        # 2. 絶対的な味の基準（純水モデル）を使って、現在の真の等価濃度を評価
                        # 【修正】まず現在の温度(T_avg)を使って、sigmaを24.6℃相当に補正する
                        temp_factor = 1.0 + TEMP_COEFF * (T_avg - T_BASE)
                        if temp_factor == 0: temp_factor = 1.0 # ゼロ割防止
                        sigma_comp = sigma / temp_factor
                        
                        # 【修正】補正済みの導電率(sigma_comp)を使って濃度を計算する
                        C_true_abs = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                        
                        # 3. 絶対的な目標値との誤差を計算
                        error = C_TARGET - C_true_abs
                        
                        # 4. 適応ゲイン調整（現在のスープでの効き目 alpha_h を加味する）
                        adaptive_ratio = ALPHA_NOMINAL / alpha_h if alpha_h > 0 else 1.0
                        kp = KP * adaptive_ratio
                        ki = KI * adaptive_ratio
                        kd = KD * adaptive_ratio
                        pid.Kp, pid.Ki, pid.Kd = kp, ki, kd
                        
                        # PIDで塩の投入量を計算
                        rate   = pid.compute(error, SALT_INTERVAL)
                        salt_g = rate * SALT_INTERVAL
                        
                        # グラフやログに記録するための濃度として、真の等価濃度をセット
                        C_hat_adaptive = C_true_abs

                    # ＝＝＝ ダッシュボード・グラフ用の表示変数の切り替え ＝＝＝
                    if CONTROL_MODE == "STR":
                        C_est = C_true_abs
                    else:
                        C_est = estimate_concentration(sigma, T_avg, alpha_h, beta_h)

                    #C_est = estimate_concentration(sigma, T_avg, alpha_h, beta_h)
                    # ＝＝＝ ★修正したのはこの4行だけです！ ＝＝＝
                    if CONTROL_MODE == "STR":
                        C_est = C_true_abs
                    else:
                        C_est = estimate_concentration(sigma, T_avg, alpha_h, beta_h)
                    # ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

                    # 塩投入
                    salt_g = min(salt_g, SALT_MAX_PER_STEP)
                    live.stop()
                    if salt_g > 0.01:
                        console.print(f"\n  [bold green]>>> {salt_g:.2f}g 投入します...[/bold green]")
                        plant.dispense_salt(salt_g)
                        total_salt += salt_g
                        last_action = f"{salt_g:.2f}g  (t={elapsed:.0f}s)"
                        console.print(f"  [green]>>> 完了  累積: {total_salt:.2f}g[/green]")
                    else:
                        last_action = f"0g  (t={elapsed:.0f}s)"
                        console.print("  [dim]>>> 今回は投入なし[/dim]")
                    live.start()

                    # ログ記録（アクション時点のみ）
                    times.append(elapsed)
                    log_sigma.append(sigma)
                    log_C.append(C_est)
                    log_alpha.append(alpha_h)
                    log_beta.append(beta_h)
                    log_salt_cum.append(total_salt)
                    log_T_avg.append(T_avg)
                    log_salt_step.append(salt_g)

                # ── ダッシュボード更新（毎秒） ──
                live.update(make_dashboard(
                    plant, manual_ec, elapsed, TOTAL_TIME,
                    C_est, total_salt, CONTROL_MODE, C_TARGET,
                    last_action, alpha_h, beta_h,
                    cycle_pos, T_avg
                ))

                time.sleep(1.0)

    except KeyboardInterrupt:
        console.print("\n[yellow]実験を中断しました[/yellow]")
    finally:
        plant.disconnect()

    if len(times) == 0:
        console.print("[red]データが記録されていません（1サイクル目の塩投入前に終了）[/red]")
        return

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # ─── CSV出力（グラフより先に保存） ───
    csv_path = f"real_experiment_{CONTROL_MODE}_{timestamp}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s",
            "T_avg_C",
            "EC_mS_cm",
            "C_est_pct",
            "C_target_pct",
            "error_pct",
            "salt_step_g",
            "salt_cumulative_g",
            "alpha_hat",
            "beta_hat",
            "control_mode",
        ])
        for i in range(len(times)):
            writer.writerow([
                f"{times[i]:.1f}",
                f"{log_T_avg[i]:.3f}",
                f"{log_sigma[i]:.4f}",
                f"{log_C[i]:.4f}",
                f"{C_TARGET:.2f}",
                f"{log_C[i] - C_TARGET:.4f}",
                f"{log_salt_step[i]:.4f}",
                f"{log_salt_cum[i]:.4f}",
                f"{log_alpha[i]:.6f}",
                f"{log_beta[i]:.6f}",
                CONTROL_MODE,
            ])
    console.print(f"[green]CSVを {csv_path} に保存しました[/green]")

    # ─── グラフ描画 ───
    if len(times) < 2:
        console.print("[yellow]データが1点のみのためグラフをスキップします[/yellow]")
        return

    try:
        t = np.array(times)
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"Real Experiment — {CONTROL_MODE}  "
                     f"({datetime.now().strftime('%Y-%m-%d %H:%M')})", fontsize=13)

        axes[0, 0].plot(t, log_sigma, color="steelblue", marker="o", ms=5)
        axes[0, 0].set(title="EC 手動入力値 [mS/cm]", xlabel="t [s]", ylabel="mS/cm")
        axes[0, 0].grid(alpha=0.3)

        axes[0, 1].plot(t, log_C, color="darkorange", marker="o", ms=5)
        axes[0, 1].axhline(C_TARGET, ls="--", color="red", label=f"Target {C_TARGET}%")
        axes[0, 1].set(title="Estimated Salinity [%]", xlabel="t [s]", ylabel="%")
        axes[0, 1].legend(); axes[0, 1].grid(alpha=0.3)

        axes[1, 0].step(t, log_salt_cum, color="green", where="post")
        axes[1, 0].set(title="Cumulative Salt [g]", xlabel="t [s]", ylabel="g")
        axes[1, 0].grid(alpha=0.3)

        axes[1, 1].plot(t, log_alpha, color="darkred",  label="α̂", marker="o", ms=5)
        axes[1, 1].plot(t, log_beta,  color="goldenrod", label="β̂", marker="o", ms=5)
        axes[1, 1].legend(); axes[1, 1].set(title="STR Parameters", xlabel="t [s]")
        axes[1, 1].grid(alpha=0.3)

        plt.tight_layout()
        png_path = f"real_experiment_{CONTROL_MODE}_{timestamp}.png"
        plt.savefig(png_path, dpi=150)
        console.print(f"[green]グラフを {png_path} に保存しました[/green]")
        plt.show()
    except Exception as e:
        console.print(f"[yellow]グラフ描画エラー: {e}[/yellow]")


if __name__ == "__main__":
    main()
