import math
import time
import serial
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

# --- 設定 ---
ROBOT_IP      = "192.168.1.200"
ESP32_PORT    = "/dev/cu.usbserial-10"   # ★ポートは環境に合わせる
ESP32_BAUD    = 115200
SALT_CMD      = "S1485"                  # 通常の塩投入（1485ms / 0.5g 相当）
SALT_CMD_LAST = "S500"                   # 最後の1回だけ（500ms）
SALT_WAIT     = 2                        # 投入完了を待つ秒数
NUM_CYCLES    = 13                        # 13ループで停止

# タイムライン（各サイクル開始からの経過秒）
CYCLE_SEC = 60
T_DIP     = 45   # state1 → state2 へ
T_SALT    = 50   # state1へ戻して塩投入 → 待ち → state3へ

speed = 1
acceleration = 0.3

# 姿勢定義
state3 = [math.radians(-8.87), math.radians(-94.31), math.radians(99.75),
          math.radians(-94.26), math.radians(-88.45), math.radians(-55.69)]
state1 = [math.radians(-10.9), math.radians(-71.8), math.radians(82.09),
          math.radians(-102.94), math.radians(-88.45), math.radians(-55.69)]
state2 = [math.radians(-10.9), math.radians(-60.50), math.radians(93.94),
          math.radians(-124.52), math.radians(-88.45), math.radians(-55.69)]


def wait_until(t0, target_s):
    """サイクル開始から target_s 秒になるまで、1秒ごとに経過秒を表示しながら待つ"""
    last = -1
    while True:
        elapsed = time.time() - t0
        if elapsed >= target_s:
            break
        sec = int(elapsed)
        if sec != last:                       # 1秒ごとに1回だけ表示
            print(f"  {sec:2d}s / {CYCLE_SEC}s", flush=True)
            last = sec
        time.sleep(0.05)


def main():
    # ロボット接続
    rtde_c = RTDEControlInterface(ROBOT_IP)
    rtde_r = RTDEReceiveInterface(ROBOT_IP)
    print("現在の姿勢:", rtde_r.getActualQ())

    # 塩ミル(ESP32)接続
    esp = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=5)
    time.sleep(2)              # ESP32が開いた瞬間にリセットされるので待つ
    esp.reset_input_buffer()

    try:
        for cycle in range(1, NUM_CYCLES + 1):
            t0 = time.time()
            esp.reset_input_buffer()   # 前サイクルのACK等をクリア
            print(f"=== サイクル {cycle} / {NUM_CYCLES} 開始 ===")

            # t=0: state3（スタート姿勢）へ
            print(f"[{time.time()-t0:.1f}s] state3へ移動")
            rtde_c.moveJ(state3, speed, acceleration)

            # t=45: state1 → すぐ state2
            wait_until(t0, T_DIP)
            print(f"[{time.time()-t0:.1f}s] state1へ移動")
            rtde_c.moveJ(state1, speed, acceleration)
            print(f"[{time.time()-t0:.1f}s] state2へ移動")
            rtde_c.moveJ(state2, speed, acceleration)

            # t=50: state1へ戻す → 塩投入 → 待ち → state3
            wait_until(t0, T_SALT)
            print(f"[{time.time()-t0:.1f}s] state1へ戻す")
            rtde_c.moveJ(state1, speed, acceleration)

            # 最後の1回だけ S500、それ以外は S1485
            salt_cmd = SALT_CMD_LAST if cycle == NUM_CYCLES else SALT_CMD
            print(f"[{time.time()-t0:.1f}s] 塩投入 -> {salt_cmd}")
            esp.write((salt_cmd + "\n").encode())

            # 投入完了を待つ ※カウント表示しながら
            salt_target = (time.time() - t0) + SALT_WAIT
            wait_until(t0, salt_target)

            print(f"[{time.time()-t0:.1f}s] state3へ移動")
            rtde_c.moveJ(state3, speed, acceleration)

            # 60秒になるまで待ってから次サイクルへ（最終ループ後は待たない）
            if cycle < NUM_CYCLES:
                wait_until(t0, CYCLE_SEC)

        print(f"=== {NUM_CYCLES}ループ完了 ===")

    except KeyboardInterrupt:
        print("\n中断しました")
    finally:
        print("停止します")
        rtde_c.stopScript()
        esp.close()


if __name__ == "__main__":
    main()