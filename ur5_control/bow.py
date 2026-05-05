import urx
import time

robot = urx.Robot("192.168.1.100")

current_joints = robot.getj()
print("現在の姿勢:", current_joints)

# ホームポジション（直立）
home = [0, -1.5708, 0, -1.5708, 0, 0]

# お辞儀ポジション
bow = [0, -1.0, -1.2, -1.5708, 0, 0]

print("ホームポジションへ移動...")
robot.movej(home, acc=0.3, vel=0.3, wait=False)
time.sleep(3)

print("お辞儀...")
robot.movej(bow, acc=0.3, vel=0.3, wait=False)
time.sleep(4)

print("戻ります...")
robot.movej(home, acc=0.3, vel=0.3, wait=False)
time.sleep(4)

print("完了！")
robot.close()
