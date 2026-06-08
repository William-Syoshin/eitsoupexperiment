"""
RealExperiment2.py
==================
実機実験用メインスクリプト（ステップ数ベース版）。

【1ステップの構造】
  0 ──────────────── 30s ──── EC入力 ──── 塩投入
  [監視・ダッシュボード更新]  [停止・EC入力・T記録]  [制御計算・塩投入]

待機中に N キーを押すと残り時間をスキップしてすぐEC入力に移る。

使い方:
    1. Arduino を USB 接続し、SERIAL_PORT を設定する
    2. python3 RealExperiment2.py
    3. 30秒ごとにプログラムが停止するのでEC値を入力する
    4. Ctrl+C で終了 → グラフ表示
"""

import csv
import re
import sys
import tty
import termios
import select
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
SERIAL_PORT       = "/dev/cu.usbserial-10"
BAUDRATE          = 115200

SALT_INTERVAL     = 30.0   # 1ステップの待機時間 [s]
MAX_STEPS         = 24     # 最大ステップ数（24×30s = 12分相当）

C_TARGET          = 1.0    # 目標塩分濃度 [%]
M_TOTAL           = 600.0  # スープ総質量 [g]
SALT_MAX_PER_STEP = 0.5    # 1ステップの最大投入量 [g]

# 純水キャリブレーション値（市販センサー mS/cm ベース）
ALPHA_NOMINAL = 8.1013
BETA_NOMINAL  = 0.1550
TEMP_COEFF    = 0.02
T_BASE        = 24.6

# ─────────────────────────────────────────
# 実機用定数・設定
# ─────────────────────────────────────────
ALPHA_MIN = ALPHA_NOMINAL / 3.0
ALPHA_MAX = ALPHA_NOMINAL * 3.0
BETA_MIN  = -5.0
BETA_MAX  = 4.0
D_ALPHA_MAX = 1.0  # 1ステップあたりの最大変化量
D_BETA_MAX  = 0.5
DEADBAND    = 0.01 # 不感帯

KP = 0.1
KI = 0.005
KD = 0.0

CONTROL_MODE = "PID"   # "STR" / "PID" / "OpenLoop"


# ─────────────────────────────────────────
# Nキー待受スレッド
# ─────────────────────────────────────────

def _key_listener(skip_event: threading.Event):
    """待機中に N/n が押されたら skip_event をセットする。"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)          # Enter不要で1文字ずつ受け取る
        while not skip_event.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if r:
                ch = sys.stdin.read(1)
                if ch.lower() == "n":
                    skip_event.set()
    except Exception:
        pass
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


# ─────────────────────────────────────────
# 手動EC入力
# ─────────────────────────────────────────

def get_manual_ec(console: Console, last_ec: float, T_now: float) -> float:
    console.print()
    console.rule("[bold yellow]EC 入力（30秒経過・プログラム停止中）[/bold yellow]")
    console.print(f"  現在の鍋温度: [cyan]{T_now:.2f} °C[/cyan]  |  目標: [cyan]{C_TARGET} %[/cyan]")
    console.print()

    while True:
        if math.isnan(last_ec):
            prompt = "  市販センサーのEC値を入力 [mS/cm]: "
        else:
            prompt = f"  EC値を入力 [mS/cm] (前回 {last_ec:.3f}, Enterでスキップ): "

        ec_str = console.input(f"[bold cyan]{prompt}[/bold cyan]").strip()
        ec_str = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", ec_str).strip()
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

def make_dashboard(plant, manual_ec, step, max_steps, remaining,
                   C_est, total_salt, control_mode, C_target,
                   last_action, alpha_h, beta_h, T_rec):

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
    progress = min(step / max(max_steps, 1), 1.0)
    bar_len  = 40
    bar      = "█" * int(bar_len * progress) + "░" * (bar_len - int(bar_len * progress))
    title_text = Text()
    title_text.append(f" Adaptive Salinity Control  [{control_mode}]  ",
                      style="bold white on navy_blue")
    title_text.append(f"  {bar}  Step {step}/{max_steps}",
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

    rem = max(remaining, 0)
    phase  = (f"[green]待機中[/green]  → {rem:.0f}s後にEC入力"
              f"  [dim]（N でスキップ）[/dim]")
    t_col  = ("red"    if plant.T_w > 55
              else "yellow" if plant.T_w > 45
              else "white")
    c_col  = ("green"  if abs(C_est - C_target) < 0.05
              else "yellow" if abs(C_est - C_target) < 0.2
              else "red")
    ec_str  = f"{manual_ec:.4f} mS/cm" if not math.isnan(manual_ec) else "[dim]未入力[/dim]"
    rec_str = f"{T_rec:.2f} °C"         if not math.isnan(T_rec)    else "[dim]—[/dim]"

    tbl.add_row("フェーズ",        phase)
    tbl.add_row("ステップ",        f"{step} / {max_steps}")
    tbl.add_row("─" * 12,         "─" * 12)
    tbl.add_row("鍋温度 (現在)",   f"[{t_col}]{plant.T_w:.2f} °C[/{t_col}]")
    tbl.add_row("鍋温度 (記録値)", rec_str)
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
        a=TEMP_COEFF, T_base=T_BASE, lam=1.0
    )
    pid = PIDController(Kp=KP, Ki=KI, Kd=KD,
                        output_min=0.0,
                        output_max=SALT_MAX_PER_STEP / SALT_INTERVAL)

    # ログ
    steps, times             = [], []
    log_sigma, log_C         = [], []
    log_salt_cum             = []
    log_alpha, log_beta      = [], []
    log_T_rec, log_salt_step = [], []

    # 状態変数
    total_salt  = 0.0
    C_est       = 0.0
    manual_ec   = float("nan")
    last_action = "—"
    alpha_h     = ALPHA_NOMINAL
    beta_h      = BETA_NOMINAL
    prev_alpha  = ALPHA_NOMINAL
    prev_beta   = BETA_NOMINAL
    T_rec       = float("nan")
    C_init_virtual = 0.0  # 💡 仮想初期濃度を記憶する変数

    start_time = time.time()
    step       = 0

    try:
        with Live(console=console, refresh_per_second=2, screen=True) as live:
            while step < MAX_STEPS:

                # ── 30秒待機（Nキーでスキップ可） ──
                skip_event = threading.Event()
                listener   = threading.Thread(
                    target=_key_listener, args=(skip_event,), daemon=True
                )
                listener.start()

                step_start = time.time()
                while True:
                    remaining = SALT_INTERVAL - (time.time() - step_start)
                    if remaining <= 0 or skip_event.is_set():
                        break
                    live.update(make_dashboard(
                        plant, manual_ec, step, MAX_STEPS, remaining,
                        C_est, total_salt, CONTROL_MODE, C_TARGET,
                        last_action, alpha_h, beta_h, T_rec
                    ))
                    time.sleep(1.0)

                # リスナー停止（ターミナル設定を復元してから console.input へ）
                skip_event.set()
                listener.join(timeout=1.0)

                # ── EC入力（プログラム停止）・入力完了の瞬間のT_wを記録 ──
                live.stop()
                manual_ec = get_manual_ec(console, manual_ec, plant.T_w)
                T_rec     = plant.T_w   # 入力完了した瞬間の温度
                elapsed   = time.time() - start_time
                console.print(f"  [dim]記録温度: {T_rec:.2f} °C[/dim]")
                live.start()

                # ── 制御計算・塩投入 ──
                sigma  = manual_ec
                salt_g = 0.0

                if CONTROL_MODE == "OpenLoop":
                    salt_g = (C_TARGET / 100.0) * M_TOTAL if total_salt == 0 else 0.0

                elif CONTROL_MODE == "PID":
                    C_hat  = estimate_concentration(sigma, T_rec, ALPHA_NOMINAL, BETA_NOMINAL)
                    error  = C_TARGET - C_hat
                    rate   = pid.compute(error, SALT_INTERVAL)
                    salt_g = rate * SALT_INTERVAL

                elif CONTROL_MODE == "STR":
                    temp_factor = 1.0 + TEMP_COEFF * (T_rec - T_BASE)
                    if temp_factor == 0:
                        temp_factor = 1.0
                    sigma_comp = sigma / temp_factor

                    # 1. 最初の1歩目で「仮想の初期塩分」を逆算
                    if total_salt < 0.01:
                        C_init_virtual = (sigma_comp - BETA_NOMINAL) / ALPHA_NOMINAL
                        prev_beta = BETA_NOMINAL 

                    # 2. X軸を「推測初期塩分＋追加量」の『仮想総濃度』にする
                    C_added = (total_salt / M_TOTAL) * 100.0
                    X_virtual_total = C_init_virtual + C_added
                    
                    # 3. 生のパラメータを学習
                    raw_alpha_h, raw_beta_h = str_unit.estimate(sigma_comp, X_virtual_total, T_rec)
                    
                    # 4. 【安全装置】変化率制限と不感帯の適用
                    d_alpha = raw_alpha_h - prev_alpha
                    d_beta  = raw_beta_h - prev_beta
                    
                    if abs(d_alpha) < DEADBAND: d_alpha = 0.0
                    if abs(d_beta)  < DEADBAND: d_beta  = 0.0
                    
                    d_alpha = max(-D_ALPHA_MAX, min(d_alpha, D_ALPHA_MAX))
                    d_beta  = max(-D_BETA_MAX,  min(d_beta,  D_BETA_MAX))
                    
                    # 更新と上下限クリップ
                    alpha_h = max(ALPHA_MIN, min(prev_alpha + d_alpha, ALPHA_MAX))
                    beta_h  = max(BETA_MIN,  min(prev_beta  + d_beta,  BETA_MAX))
                    
                    # 内部状態を強制更新
                    prev_alpha, prev_beta = alpha_h, beta_h
                    str_unit.theta[0, 0] = alpha_h
                    str_unit.theta[1, 0] = beta_h
                    
                    # ゲインの適応調整
                    adaptive_ratio = ALPHA_NOMINAL / alpha_h
                    adaptive_ratio = min(max(adaptive_ratio, 0.3), 3.0)
                    pid.Kp = KP * adaptive_ratio
                    pid.Ki = KI * adaptive_ratio
                    pid.Kd = KD * adaptive_ratio
                    
                    # 5. X軸をズラしたので、beta_h を直接使って総濃度を計算
                    if total_salt < 0.01:
                        C_true_abs = C_init_virtual
                    else:
                        C_true_abs = (sigma_comp - beta_h) / alpha_h
                    
                    error = C_TARGET - C_true_abs
                    salt_g = pid.compute(error, SALT_INTERVAL) * SALT_INTERVAL

                # 表示・ログ用濃度：STRは推定値、それ以外は固定値
                temp_factor_disp = 1.0 + TEMP_COEFF * (plant.T_w - T_BASE)
                if temp_factor_disp == 0: temp_factor_disp = 1.0
                sigma_comp_disp = sigma / temp_factor_disp
                
                if CONTROL_MODE == "STR" and total_salt >= 0.01:
                    C_est = (sigma_comp_disp - beta_h) / alpha_h
                else:
                    C_est = (sigma_comp_disp - BETA_NOMINAL) / ALPHA_NOMINAL

                # 塩投入
                salt_g = min(salt_g, SALT_MAX_PER_STEP)
                live.stop()
                if salt_g > 0.01:
                    console.print(f"\n  [bold green]>>> {salt_g:.2f}g 投入します...[/bold green]")
                    plant.dispense_salt(salt_g)
                    total_salt += salt_g
                    last_action = f"{salt_g:.2f}g  (Step {step + 1})"
                    console.print(f"  [green]>>> 完了  累積: {total_salt:.2f}g[/green]")
                else:
                    last_action = f"0g  (Step {step + 1})"
                    console.print("  [dim]>>> 今回は投入なし[/dim]")
                live.start()

                # ログ記録
                step += 1
                steps.append(step)
                times.append(elapsed)
                log_sigma.append(sigma)
                log_C.append(C_est)
                log_alpha.append(alpha_h)
                log_beta.append(beta_h)
                log_salt_cum.append(total_salt)
                log_T_rec.append(T_rec)
                log_salt_step.append(salt_g)

    except KeyboardInterrupt:
        console.print("\n[yellow]実験を中断しました[/yellow]")
    finally:
        plant.disconnect()

    if len(steps) == 0:
        console.print("[red]データが記録されていません[/red]")
        return

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # ─── CSV出力（グラフより先に保存） ───
    csv_path = f"real_experiment_{CONTROL_MODE}_{timestamp}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "step",
            "time_s",
            "T_rec_C",
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
        for i in range(len(steps)):
            writer.writerow([
                steps[i],
                f"{times[i]:.1f}",
                f"{log_T_rec[i]:.3f}",
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
    if len(steps) < 2:
        console.print("[yellow]データが1点のみのためグラフをスキップします[/yellow]")
        return

    try:
        s = np.array(steps)
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"Real Experiment — {CONTROL_MODE}  "
                     f"({datetime.now().strftime('%Y-%m-%d %H:%M')})", fontsize=13)

        axes[0, 0].plot(s, log_sigma, color="steelblue", marker="o", ms=5)
        axes[0, 0].set(title="EC 手動入力値 [mS/cm]", xlabel="Step", ylabel="mS/cm")
        axes[0, 0].grid(alpha=0.3)

        axes[0, 1].plot(s, log_C, color="darkorange", marker="o", ms=5)
        axes[0, 1].axhline(C_TARGET, ls="--", color="red", label=f"Target {C_TARGET}%")
        axes[0, 1].set(title="Estimated Salinity [%]", xlabel="Step", ylabel="%")
        axes[0, 1].legend()
        axes[0, 1].grid(alpha=0.3)

        axes[1, 0].step(s, log_salt_cum, color="green", where="post")
        axes[1, 0].set(title="Cumulative Salt [g]", xlabel="Step", ylabel="g")
        axes[1, 0].grid(alpha=0.3)

        axes[1, 1].plot(s, log_alpha, color="darkred",   label="α̂", marker="o", ms=5)
        axes[1, 1].plot(s, log_beta,  color="goldenrod", label="β̂", marker="o", ms=5)
        axes[1, 1].legend()
        axes[1, 1].set(title="STR Parameters", xlabel="Step")
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
