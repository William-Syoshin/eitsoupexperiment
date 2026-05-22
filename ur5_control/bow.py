import math
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

ROBOT_IP = "192.168.1.200"

rtde_c = RTDEControlInterface(ROBOT_IP)
rtde_r = RTDEReceiveInterface(ROBOT_IP)

current_joints = rtde_r.getActualQ()
print("現在の姿勢:", current_joints)

# ホームポジション（直立）
home = [0, -1.5708, 0, -1.5708, 0, 0]

# お辞儀ポジション
state1 = [math.radians(0), math.radians(-70), math.radians(85.36),
            math.radians(-108), math.radians(-90), math.radians(0)]

state2 = [math.radians(0), math.radians(-60), math.radians(93.5),
            math.radians(-125), math.radians(-90), math.radians(0)]

speed = 1
acceleration = 0.3

print("ホームポジションへ移動...")
rtde_c.moveJ(home, speed, acceleration)
for i in range(10):
    print("お辞儀...")
    rtde_c.moveJ(state1, speed, acceleration)

    print("お辞儀...")
    rtde_c.moveJ(state2, speed, acceleration)

print("戻ります...")
rtde_c.moveJ(home, speed, acceleration)

print("完了！")
rtde_c.stopScript()
