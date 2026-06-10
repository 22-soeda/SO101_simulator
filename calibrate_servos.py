"""サーボのキャリブレーション(オフセット・可動範囲)を対話的に設定する。

手順:
  1. 各関節のサーボを、シミュレーター上の論理角度0度(基準姿勢)に
     対応する実機の姿勢に手で動かしておく。
  2. このスクリプトを実行する。接続されているサーボについて、現在の
     サーボ位置を読み取り、回転方向と現在の論理角度[deg]を入力すると、
     home_position(論理角度0度に対応するサーボ位置)を逆算して保存する。
  3. JOINT_LIMITS_DEG (so101_kinematics.py) から、サーボに送ってよい
     位置の範囲(position_min/max)も合わせて計算・保存する。

設定値は servo_calibration.json に保存され、simulator_ik_sync.py で
使われる。

使い方:
    python calibrate_servos.py
"""

import servo_config as config
from servo_sync import ServoSync
from calibration import load_calibration, save_calibration, CALIBRATION_FILE
from so101_kinematics import JOINT_NAMES, JOINT_LIMITS_DEG


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

    steps_per_deg = config.STEPS_PER_REV / 360.0

    for joint_name in JOINT_NAMES:
        scs_id = config.JOINT_TO_SERVO_ID[joint_name]
        if scs_id not in connected:
            continue

        position = sync.read_position(scs_id)
        entry = calib[joint_name]

        print(f"\n--- {joint_name} (ID {scs_id}) ---")
        print(f"現在のサーボ位置: {position}")

        direction_str = input(
            f"回転方向 (1: サーボ位置増加=論理角度+方向, -1: 逆) "
            f"[{entry['direction']}]: "
        ).strip()
        direction = int(direction_str) if direction_str else entry["direction"]

        angle_str = input("現在の論理角度[deg] (基準姿勢なら0) [0]: ").strip()
        angle_deg = float(angle_str) if angle_str else 0.0

        home_position = position - direction * angle_deg * steps_per_deg
        home_position = int(round(home_position))

        lo, hi = JOINT_LIMITS_DEG[joint_name]
        pos_at_lo = home_position + direction * lo * steps_per_deg
        pos_at_hi = home_position + direction * hi * steps_per_deg
        position_min = int(round(min(pos_at_lo, pos_at_hi)))
        position_max = int(round(max(pos_at_lo, pos_at_hi)))
        position_min = max(position_min, config.POSITION_MIN)
        position_max = min(position_max, config.POSITION_MAX)

        entry["direction"] = direction
        entry["home_position"] = home_position
        entry["position_min"] = position_min
        entry["position_max"] = position_max

        print(f"  home_position = {home_position}")
        print(f"  position_min/max = {position_min} / {position_max}")

    save_calibration(calib)
    sync.close()
    print(f"\nキャリブレーションを {CALIBRATION_FILE} に保存しました。")


if __name__ == "__main__":
    main()
