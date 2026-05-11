"""
Teensyシリアル出力をCSVファイルに保存するスクリプト。
8電極・隣接パターンのEIT計測データ（40値/フレーム）に対応。

Usage:
    python serial_to_csv.py                      # ポート自動検出
    python serial_to_csv.py /dev/cu.usbmodem1234
    python serial_to_csv.py /dev/cu.usbmodem1234 output.csv

Requirements:
    pip install pyserial
"""

import sys
import glob
import csv
import time
from datetime import datetime

import serial

# ─── settings ────────────────────────────────────────────────────────────────
BAUD_RATE = 115200
N_EL      = 8
N_MEAS    = N_EL * (N_EL - 3)   # 40
# ─────────────────────────────────────────────────────────────────────────────


def find_port():
    candidates = (
        glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
    )
    if not candidates:
        raise RuntimeError("シリアルポートが見つかりません。USBを確認してください。")
    print(f"[serial] 使用ポート: {candidates[0]}")
    return candidates[0]


def parse_line(line: str, n_cols: int = 0):
    """カンマ区切り行をfloatリストに変換。データ行でなければNoneを返す。
    n_cols=0 のときは列数を問わず有効なfloat行を受け入れる。"""
    line = line.strip()
    if not line or line.startswith("["):
        return None
    parts = line.split(",")
    if n_cols and len(parts) != n_cols:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def make_header(n_cols: int):
    if n_cols == N_MEAS:
        # 40値: tx_pair * 5 + k ラベル
        labels = []
        meas_per_drive = N_EL - 3
        for tx in range(N_EL):
            for k in range(meas_per_drive):
                labels.append(f"tx{tx}_k{k}")
    else:
        labels = [f"ch{i}" for i in range(n_cols)]
    return ["timestamp"] + labels


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = sys.argv[2] if len(sys.argv) > 2 else f"eit_{timestamp_str}.csv"

    print(f"[csv]    保存先: {output_file}")
    print("[info]   Ctrl+C で停止")

    frame_count = 0

    def open_port(port):
        """ポートを開き、先頭の壊れた行を読み捨ててから返す。"""
        for attempt in range(5):
            try:
                ser = serial.Serial(port, BAUD_RATE, timeout=2)
                time.sleep(3)          # Teensyリセット完了を待つ
                ser.reset_input_buffer()
                # 先頭 5行を読み捨て（リセット直後の不完全な行を除去）
                for _ in range(5):
                    ser.readline()
                ser.reset_input_buffer()
                return ser
            except serial.SerialException as e:
                print(f"[retry]  接続試行 {attempt+1}/5: {e}")
                time.sleep(2)
        raise RuntimeError("シリアルポートへの接続に失敗しました。")

    ser = open_port(port)
    print(f"[serial] 接続成功: {port}")

    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)

        n_cols = 0
        col_candidate = 0
        col_streak = 0
        STREAK_REQUIRED = 3   # 同じ列数が連続3行で確定

        try:
            while True:
                try:
                    raw = ser.readline().decode("utf-8", errors="ignore")
                except serial.SerialException:
                    print("\n[warn]   切断検出。再接続します...")
                    ser.close()
                    time.sleep(3)
                    ser = open_port(port)
                    print("[serial] 再接続成功")
                    n_cols = 0
                    col_candidate = 0
                    col_streak = 0
                    continue

                values = parse_line(raw, 0)

                if values is None or len(values) < 2:
                    stripped = raw.strip()
                    if stripped:
                        print(f"[teensy] {stripped}")
                    continue

                # 列数確定フェーズ：同じ列数がSTREAK_REQUIRED回連続したら確定
                if n_cols == 0:
                    n = len(values)
                    if n == col_candidate:
                        col_streak += 1
                    else:
                        col_candidate = n
                        col_streak = 1
                    if col_streak >= STREAK_REQUIRED:
                        n_cols = col_candidate
                        print(f"[auto]   列数を {n_cols} として確定しました")
                        writer.writerow(make_header(n_cols))
                        f.flush()
                    continue   # 確定前は記録しない

                if len(values) != n_cols:
                    continue

                ts = time.time()
                writer.writerow([f"{ts:.6f}"] + [f"{v:.6f}" for v in values])
                f.flush()

                frame_count += 1
                print(f"\r[rec]    {frame_count} フレーム保存済み", end="", flush=True)

        except KeyboardInterrupt:
            print(f"\n[exit]   終了 — 合計 {frame_count} フレームを {output_file} に保存しました")
        finally:
            ser.close()


if __name__ == "__main__":
    main()
