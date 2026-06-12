"""ホームポジション(シミュレーター起動時の姿勢)の読み書き。

home_pose_deg : 各関節の論理角度[deg] (calibrate_homing.pyで定義された
                論理角度0度の姿勢から、何度動かした姿勢かを表す)

設定値は home_position.json に保存され、各シミュレーターの起動姿勢として
使われる。calibrate_homing.py で対話的に設定する。
"""

import json
import os

from so101_kinematics import JOINT_NAMES

HOME_POSITION_FILE = os.path.join(os.path.dirname(__file__), "home_position.json")


def default_home_pose():
    return {name: 0.0 for name in JOINT_NAMES}


def load_home_pose():
    pose = default_home_pose()
    if os.path.exists(HOME_POSITION_FILE):
        with open(HOME_POSITION_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        for name, value in data.items():
            if name in pose:
                pose[name] = float(value)
    return pose


def save_home_pose(pose):
    with open(HOME_POSITION_FILE, "w", encoding="utf-8") as f:
        json.dump(pose, f, indent=2, ensure_ascii=False)
