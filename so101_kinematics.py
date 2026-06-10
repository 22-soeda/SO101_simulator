"""SO-101アームの順運動学(Forward Kinematics)モデル。

各関節のオフセット(origin xyz/rpy)・回転軸・可動範囲は、公式リポジトリ
TheRobotStudio/SO-ARM100 の
Simulation/SO101/so101_new_calib.urdf
から取得した値をそのまま使用している(3D的なオフセットを忠実に再現)。

SO-101は以下の6つの軸(サーボ)で構成される:
    1. shoulder_pan  : ベース回転
    2. shoulder_lift : 肩の上下
    3. elbow_flex    : 肘の曲げ
    4. wrist_flex    : 手首の曲げ
    5. wrist_roll    : 手首の回転
    6. gripper       : グリッパーの開閉

各関節の回転軸はURDF上ではすべてローカルZ軸 (axis="0 0 1") だが、
joint origin の rpy によって親リンクに対するローカル座標系自体が
回転しているため、ワールド座標系で見ると軸の向きは関節ごとに異なる。
"""

import numpy as np

# --- 各関節のURDF origin (xyz [m], rpy [rad]) と可動範囲 (rad) ---
# Simulation/SO101/so101_new_calib.urdf より
JOINT_PARAMS = {
    "shoulder_pan": {
        "xyz": (0.0388353, -8.97657e-09, 0.0624),
        "rpy": (3.14159, 4.18253e-17, -3.14159),
        "limits_rad": (-1.91986, 1.91986),
    },
    "shoulder_lift": {
        "xyz": (-0.0303992, -0.0182778, -0.0542),
        "rpy": (-1.5708, -1.5708, 0.0),
        "limits_rad": (-1.74533, 1.74533),
    },
    "elbow_flex": {
        "xyz": (-0.11257, -0.028, 1.73763e-16),
        "rpy": (-3.63608e-16, 8.74301e-16, 1.5708),
        "limits_rad": (-1.69, 1.69),
    },
    "wrist_flex": {
        "xyz": (-0.1349, 0.0052, 3.62355e-17),
        "rpy": (4.02456e-15, 8.67362e-16, -1.5708),
        "limits_rad": (-1.65806, 1.65806),
    },
    "wrist_roll": {
        "xyz": (5.55112e-17, -0.0611, 0.0181),
        "rpy": (1.5708, 0.0486795, 3.14159),
        "limits_rad": (-2.74385, 2.84121),
    },
    "gripper": {
        "xyz": (0.0202, 0.0188, -0.0234),
        "rpy": (1.5708, -5.24284e-08, -1.41553e-15),
        "limits_rad": (-0.174533, 1.74533),
    },
}

# gripper_link -> TCP (gripper_frame_link) の固定変換 (fixed joint)
GRIPPER_FRAME = {
    "xyz": (-0.0079, -0.000218121, -0.0981274),
    "rpy": (0.0, 3.14159, 0.0),
}

JOINT_NAMES = list(JOINT_PARAMS.keys())

JOINT_LIMITS_DEG = {
    name: tuple(np.degrees(p["limits_rad"]).tolist())
    for name, p in JOINT_PARAMS.items()
}

# グリッパー指の長さ・最大開き幅 [m] (見た目用の概算値)
FINGER_LENGTH = 0.05
FINGER_MAX_OFFSET = 0.025


def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1],
    ])


def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1],
    ])


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def translate(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def rpy_matrix(roll, pitch, yaw):
    """URDFのorigin rpyと同じ規約: R = Rz(yaw) * Ry(pitch) * Rx(roll)。"""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def joint_transform(name, theta):
    """関節originの固定変換 + 関節角度theta[rad]によるZ軸回転(URDF axis="0 0 1")。"""
    p = JOINT_PARAMS[name]
    return translate(*p["xyz"]) @ rpy_matrix(*p["rpy"]) @ rot_z(theta)


def clamp_angles(angles_deg):
    """各軸の角度を可動範囲内に収める。"""
    clamped = []
    for name, value in zip(JOINT_NAMES, angles_deg):
        lo, hi = JOINT_LIMITS_DEG[name]
        clamped.append(min(max(value, lo), hi))
    return clamped


def forward_kinematics(angles_deg):
    """6軸の角度[deg]からリンクの関節点列・手先姿勢・指の線分を計算する。

    戻り値:
        points: 各関節位置 (Nx3 array) -- base_link原点からTCP(グリッパー先端)まで
        end_effector_T: TCP(グリッパー先端中心)の同次変換行列 (4x4)
        fingers: グリッパー指先端2本の線分 [(p_start, p_end), ...]
    """
    angles_rad = np.radians(angles_deg)

    T = np.eye(4)
    points = [T[:3, 3].copy()]

    # 1〜5: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
    for name, theta in zip(JOINT_NAMES[:5], angles_rad[:5]):
        T = T @ joint_transform(name, theta)
        points.append(T[:3, 3].copy())

    # T: wrist_roll軸の位置・姿勢 (= gripper_link フレーム)
    T_gripper_link = T

    # TCP (グリッパー先端中心、固定オフセット)
    end_effector_T = T_gripper_link @ translate(*GRIPPER_FRAME["xyz"]) @ rpy_matrix(*GRIPPER_FRAME["rpy"])

    # アームのリンクとグリッパーをつなぐためTCP位置も関節点列に追加
    points.append(end_effector_T[:3, 3].copy())

    # グリッパー指 (見た目用)
    # 指はwrist_roll関節(5軸目)の回転軸方向に伸ばし、グリッパー角度に
    # 応じて回転軸に垂直な方向に開閉する。
    grip_deg = angles_deg[5]
    lo, hi = JOINT_LIMITS_DEG["gripper"]
    grip_norm = (grip_deg - lo) / (hi - lo)
    offset = FINGER_MAX_OFFSET * grip_norm

    base = end_effector_T[:3, 3]

    # wrist_roll回転軸 (T_gripper_linkのZ軸)。TCP方向を向くように符号を揃える。
    finger_axis = T_gripper_link[:3, 2]
    if np.dot(finger_axis, base - T_gripper_link[:3, 3]) < 0:
        finger_axis = -finger_axis

    open_axis = T_gripper_link[:3, 1]

    fingers = []
    for sign in (+1, -1):
        start = base + sign * offset * open_axis
        end = start + FINGER_LENGTH * finger_axis
        fingers.append((start, end))

    return np.array(points), end_effector_T, fingers
