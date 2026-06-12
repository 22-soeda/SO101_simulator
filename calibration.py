"""サーボのキャリブレーション値(オフセット・可動範囲)の読み書き。"""

import json
import os

from so101_kinematics import JOINT_NAMES, JOINT_LIMITS_DEG
import servo_config as config

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "servo_calibration.json")

# home_position : 論理角度0度のときのサーボ位置
# direction     : +1ならサーボ位置増加方向が論理角度+方向と一致、-1なら逆
# position_min/max : このサーボに指示してよい位置の範囲 (ソフトリミット)
DEFAULT_ENTRY = {
    "home_position": 2048,
    "direction": 1,
    "position_min": config.POSITION_MIN,
    "position_max": config.POSITION_MAX,
}


def default_calibration():
    return {name: dict(DEFAULT_ENTRY) for name in JOINT_NAMES}


def load_calibration():
    calib = default_calibration()
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        for name, entry in data.items():
            if name in calib:
                calib[name].update(entry)
    return calib


def save_calibration(calib):
    with open(CALIBRATION_FILE, "w", encoding="utf-8") as f:
        json.dump(calib, f, indent=2, ensure_ascii=False)


def _position_to_angle_deg(position, home_position, direction):
    """サーボ位置を論理角度[deg]に変換する(0/4095境界をまたぐ場合に対応)。"""
    diff = (position - home_position) % config.STEPS_PER_REV
    if diff > config.STEPS_PER_REV / 2:
        diff -= config.STEPS_PER_REV
    return diff / direction / (config.STEPS_PER_REV / 360.0)


def calibrated_joint_limits_deg(calib=None):
    """各関節の有効な角度範囲[deg]を返す。

    servo_calibration.jsonのposition_min/position_max(サーボに送信してよい
    範囲)を論理角度に変換し、so101_kinematics.JOINT_LIMITS_DEG(URDFの可動
    範囲)との共通範囲を返す。シミュレーター・IKはこの範囲内で動作させる。
    """
    if calib is None:
        calib = load_calibration()

    limits = {}
    for name in JOINT_NAMES:
        entry = calib[name]
        urdf_lo, urdf_hi = JOINT_LIMITS_DEG[name]

        # position_min/maxが(ほぼ)全周分の場合は未設定とみなし、URDFの
        # 可動範囲のみを使う(そのまま変換すると0/4095境界で反転した
        # 範囲になってしまうため)。
        position_range = entry["position_max"] - entry["position_min"]
        if position_range >= config.STEPS_PER_REV - 1:
            limits[name] = (urdf_lo, urdf_hi)
            continue

        a_min = _position_to_angle_deg(entry["position_min"], entry["home_position"], entry["direction"])
        a_max = _position_to_angle_deg(entry["position_max"], entry["home_position"], entry["direction"])
        lo, hi = min(a_min, a_max), max(a_min, a_max)

        limits[name] = (max(lo, urdf_lo), min(hi, urdf_hi))

    return limits
