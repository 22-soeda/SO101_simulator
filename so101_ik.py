"""SO-101アームの数値逆運動学(IK)。

shoulder_pan, shoulder_lift, elbow_flex, wrist_flex の4関節を、
TCP(グリッパー先端)の目標位置(x, y, z)と、グリッパーが向く角度
(水平面に対するピッチ角)に一致させるように反復計算で求める。

wrist_roll(手首回転)とgripper(開閉)はIKの対象外で、別途直接指定する。
"""

import numpy as np

from so101_kinematics import (
    JOINT_NAMES,
    JOINT_LIMITS_DEG,
    forward_kinematics,
)

# IKで解く4関節 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex)
IK_JOINT_NAMES = JOINT_NAMES[:4]

# 数値微分のステップ幅 [deg]
_FD_EPS_DEG = 0.5

# 疑似逆行列の特異値カットオフ (特異姿勢付近での発散を防ぐ)
_PINV_RCOND = 1e-3

# 1反復あたりの角度更新量の上限 [deg]
_MAX_STEP_DEG = 15.0


def end_effector_pose(q4_deg, wrist_roll_deg, gripper_deg):
    """4関節の角度[deg]からTCP位置[x,y,z]とピッチ角[rad]を返す。

    ピッチ角は、グリッパーの指が向く方向(wrist_roll回転軸方向)と
    水平面(XY平面)とのなす角 (上向きが正)。
    """
    angles = list(q4_deg) + [wrist_roll_deg, gripper_deg]
    _, end_effector_T, fingers = forward_kinematics(angles)

    pos = end_effector_T[:3, 3]

    finger_dir = fingers[0][1] - fingers[0][0]
    finger_dir = finger_dir / np.linalg.norm(finger_dir)
    horizontal = np.hypot(finger_dir[0], finger_dir[1])
    pitch = np.arctan2(finger_dir[2], horizontal)

    return np.array([pos[0], pos[1], pos[2], pitch])


def _numerical_jacobian(q4_deg, wrist_roll_deg, gripper_deg):
    g0 = end_effector_pose(q4_deg, wrist_roll_deg, gripper_deg)
    J = np.zeros((4, 4))
    for i in range(4):
        q_perturbed = np.array(q4_deg, dtype=float)
        q_perturbed[i] += _FD_EPS_DEG
        g1 = end_effector_pose(q_perturbed, wrist_roll_deg, gripper_deg)
        J[:, i] = np.radians(g1 - g0) / np.radians(_FD_EPS_DEG)
    return J, g0


def _error_norm(err):
    """位置誤差[m]と姿勢誤差[rad]を合成したスカラー誤差。"""
    return float(np.linalg.norm(err))


def _clamp(q):
    q = np.array(q, dtype=float)
    for i, name in enumerate(IK_JOINT_NAMES):
        lo, hi = JOINT_LIMITS_DEG[name]
        q[i] = min(max(q[i], lo), hi)
    return q


def solve_ik(target, q4_init_deg, wrist_roll_deg, gripper_deg,
              max_iters=50, tol_pos=1e-4, tol_pitch=1e-3):
    """目標[x,y,z,pitch(rad)]に最も近い4関節角度[deg]を反復計算で求める。

    ヤコビアンの疑似逆行列を使ったGauss-Newton法。特異姿勢付近では振動
    することがあるため、反復中で最も誤差が小さかった角度を記録して返す
    (収束しない場合(可動範囲外など)も、それまでで最も近づいた角度になる)。
    """
    target = np.array(target, dtype=float)

    q = _clamp(q4_init_deg)
    best_q, best_err_norm = q.copy(), None

    for _ in range(max_iters):
        J, g = _numerical_jacobian(q, wrist_roll_deg, gripper_deg)
        err = target - g
        err_norm = _error_norm(err)

        if best_err_norm is None or err_norm < best_err_norm:
            best_q, best_err_norm = q.copy(), err_norm

        if np.linalg.norm(err[:3]) < tol_pos and abs(err[3]) < tol_pitch:
            break

        delta_rad = np.linalg.pinv(J, rcond=_PINV_RCOND) @ err
        delta_deg = np.degrees(delta_rad)

        step = np.max(np.abs(delta_deg))
        if step > _MAX_STEP_DEG:
            delta_deg *= _MAX_STEP_DEG / step

        q = _clamp(q + delta_deg)

    return best_q
