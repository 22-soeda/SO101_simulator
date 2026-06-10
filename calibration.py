"""サーボのキャリブレーション値(オフセット・可動範囲)の読み書き。"""

import json
import os

from so101_kinematics import JOINT_NAMES
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
