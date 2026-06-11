"""シミュレーターの起動姿勢(ホームポジション)を対話的に設定する。

calibrate_servos.pyによって、サーボの生位置と論理角度0度(初期姿勢)との
対応(home_position/direction)はすでに分かっている。このスクリプトでは、
そこからさらに「シミュレーター起動時にどの姿勢から始めるか」を、
各関節の論理角度[deg]として決める。

手順:
  1. このスクリプトを実行する。接続されているサーボのトルクをOFFにし、
     各関節の現在の論理角度[deg]を読み取り続けて表示する
     (脱力状態なので、アームを手で動かして好きな姿勢にできる)。
  2. ホームポジションにしたい姿勢になったらCtrl+Cで読み取りを停止する。
  3. そのときの各関節の論理角度[deg]を home_position.json に
     home_pose_deg として保存する。

設定値は home_position.json に保存され、simulator.py / simulator_ik.py の
起動姿勢として使われる。

使い方:
    python set_home_position.py
"""

import time

import servo_config as config
from servo_sync import ServoSync
from calibration import load_calibration
from home_position import load_home_pose, save_home_pose, HOME_POSITION_FILE
from so101_kinematics import JOINT_NAMES, clamp_angles

READ_INTERVAL = 0.2


def main():
    calib = load_calibration()
    sync = ServoSync(calibration=calib)
    sync.open()

    connected = sync.scan()
    print(f"接続されているサーボID: {sorted(connected)}")
    if not connected:
        print("サーボが見つかりませんでした。配線・電源・COMポート設定を確認してください。")
        sync.close()
        return

    # 脱力状態にして、手でアームを動かせるようにする
    for joint_name in JOINT_NAMES:
        scs_id = config.JOINT_TO_SERVO_ID[joint_name]
        if scs_id in connected:
            sync.set_torque_enable(scs_id, False)

    print("\nサーボのトルクをOFFにしました。")
    print("アームを手で動かして、ホームポジションにしたい姿勢にしてください。")
    print("Ctrl+C で読み取りを停止し、その姿勢を保存します。\n")

    angles = load_home_pose()

    try:
        while True:
            parts = []
            for joint_name in JOINT_NAMES:
                scs_id = config.JOINT_TO_SERVO_ID[joint_name]
                if scs_id not in connected:
                    parts.append(f"{joint_name}=--")
                    continue
                position = sync.read_position(scs_id)
                if position is None:
                    parts.append(f"{joint_name}=??")
                    continue
                angles[joint_name] = sync.position_to_angle(joint_name, position)
                parts.append(f"{joint_name}={angles[joint_name]:7.1f}")
            print("\r" + "  ".join(parts), end="", flush=True)
            time.sleep(READ_INTERVAL)
    except KeyboardInterrupt:
        print()

    clamped = clamp_angles([angles[name] for name in JOINT_NAMES])
    angles = dict(zip(JOINT_NAMES, clamped))

    save_home_pose(angles)
    sync.close()

    print(f"\nホームポジションを {HOME_POSITION_FILE} に保存しました。")
    for joint_name in JOINT_NAMES:
        print(f"  {joint_name}: {angles[joint_name]:.1f} deg")


if __name__ == "__main__":
    main()
