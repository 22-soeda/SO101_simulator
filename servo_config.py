"""FEETECHサーボ通信の設定。"""

# 接続設定
DEVICE_NAME = "COM12"
BAUDRATE = 1000000

# プロトコル設定 (STS/SMSシリーズ = 0, SCSシリーズ = 1)
PROTOCOL_END = 0

# 制御テーブルアドレス (SMS/STS系)
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_ACC = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_PRESENT_POSITION = 56

# サーボの位置レンジ (12bit, 1回転 = 4096ステップ = 360度)
POSITION_MIN = 0
POSITION_MAX = 4095
STEPS_PER_REV = 4096

# 角度指示時の既定の移動速度・加速度
MOVING_SPEED = 1000
MOVING_ACC = 50

# 関節名 -> サーボID (SO-101の標準的な割り当て)
JOINT_TO_SERVO_ID = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
