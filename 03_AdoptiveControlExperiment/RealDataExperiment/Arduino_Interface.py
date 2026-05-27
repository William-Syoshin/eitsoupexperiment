"""
Arduino_Interface.py
====================
ESP32 との通信クラス。
温度計測（MAX31865 / DHT22）とソルトミル制御を担当。
EC は市販センサーで手動計測するため、このクラスでは扱わない。

シリアルプロトコル:
    Arduino → PC : "T:<pot_C>,RT:<room_C>\\n"
    PC → Arduino : "S<milliseconds>\\n"
    Arduino → PC : "ACK:SALT_DONE\\n"
"""

import serial
import threading
import time
from collections import deque

# ── ソルトミルのキャリブレーション (実験値) ──
# y = 0.3291 * x + 0.0227   (y: 塩[g], x: 時間[s])
CALIB_SLOPE     = 0.3291   # [g/s]
CALIB_INTERCEPT = 0.0227   # [g]

# ── モーター出力補正係数 ──
# 実測: キャリブレーション通りの時間で動かすと実際には 0.65 倍しか出ない
# そのため計算した稼働時間を 1/0.65 倍に延ばして補正する
MOTOR_CORRECTION = 1.0 / 0.65


def grams_to_ms(salt_g: float) -> int:
    """投入したい塩の量[g]をモーター稼働時間[ms]に変換する"""
    if salt_g <= CALIB_INTERCEPT:
        return 0
    duration_s = (salt_g - CALIB_INTERCEPT) / CALIB_SLOPE
    return int(duration_s * 1000 * MOTOR_CORRECTION)


class ArduinoInterface:
    """
    ESP32 とシリアル通信するクラス。
    温度の取得とソルトミル制御のみ担当する。
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        self.port     = port
        self.baudrate = baudrate
        self.timeout  = timeout

        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()

        self._T_w    = float("nan")   # 鍋の温度 [°C]
        self._T_room = float("nan")   # 室温 [°C]

        # シリアルモニタ用：受信した生データを最新30行分保持
        self._raw_lines: deque[str] = deque(maxlen=30)

        self._reading_thread: threading.Thread | None = None
        self._running = False

    # ──────────────────────────────────────────
    # 接続 / 切断
    # ──────────────────────────────────────────

    def connect(self):
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2.0)  # ESP32リセット待ち
        self._ser.reset_input_buffer()
        self._running = True
        self._reading_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reading_thread.start()
        print(f"[Arduino] Connected: {self.port} @ {self.baudrate} baud")

    def disconnect(self):
        self._running = False
        if self._reading_thread:
            self._reading_thread.join(timeout=3.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[Arduino] Disconnected")

    # ──────────────────────────────────────────
    # プロパティ
    # ──────────────────────────────────────────

    @property
    def T_w(self) -> float:
        """鍋の温度 [°C]"""
        with self._lock:
            return self._T_w

    @property
    def T_room(self) -> float:
        """室温 [°C]"""
        with self._lock:
            return self._T_room

    @property
    def raw_lines(self) -> list[str]:
        """シリアルモニタ表示用：受信した生データの最新行リスト"""
        with self._lock:
            return list(self._raw_lines)

    # ──────────────────────────────────────────
    # 塩投入
    # ──────────────────────────────────────────

    def dispense_salt(self, salt_g: float) -> int:
        """
        指定グラム数の塩を投入する。モーターが止まるまでブロック。

        Returns:
            実際の稼働時間 [ms]
        """
        duration_ms = grams_to_ms(salt_g)
        if duration_ms <= 0:
            return 0

        cmd = f"S{duration_ms}\n"
        if self._ser and self._ser.is_open:
            self._ser.write(cmd.encode())
            # モーター稼働時間 + 余裕1秒 待機（最大1g ≒ 3s + 1s = 4s）
            time.sleep(duration_ms / 1000.0 + 1.0)
            return duration_ms
        return 0

    # ──────────────────────────────────────────
    # バックグラウンド受信ループ
    # ──────────────────────────────────────────

    def _read_loop(self):
        while self._running:
            try:
                if self._ser and self._ser.in_waiting:
                    line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                    self._parse_line(line)
                else:
                    time.sleep(0.05)
            except Exception as e:
                time.sleep(0.1)

    def _parse_line(self, line: str):
        if not line:
            return

        with self._lock:
            self._raw_lines.append(line)

        if line.startswith("ACK:") or line.startswith("ERR:") or line == "READY":
            return

        try:
            parts = {}
            for token in line.split(","):
                key, val = token.split(":")
                parts[key.strip()] = val.strip()

            with self._lock:
                if "T"  in parts: self._T_w    = float(parts["T"])
                if "RT" in parts: self._T_room = float(parts["RT"])
        except Exception:
            pass
