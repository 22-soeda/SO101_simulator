"""SO-101アーム IKシミュレーター + 実機サーボ同期版。

simulator_ik.py のスライダー操作で計算された関節角度を、接続されている
FEETECHサーボへリアルタイムに送信する。未接続のサーボIDは無視されるため、
一部の関節のみ接続している場合でもそのまま使える。

事前に calibrate_servos.py でキャリブレーションしておくこと。

使い方:
    python simulator_ik_sync.py
"""

import matplotlib.pyplot as plt

from simulator_ik import IKSimulator
from so101_ik import IK_JOINT_NAMES
from servo_sync import ServoSync


class SyncIKSimulator(IKSimulator):
    def __init__(self, servo_sync):
        self.servo_sync = servo_sync
        super().__init__()

    def redraw(self):
        super().redraw()

        angles_deg = dict(zip(IK_JOINT_NAMES, self.q4))
        angles_deg["wrist_roll"] = self.wrist_roll
        angles_deg["gripper"] = self.gripper
        self.servo_sync.send_angles(angles_deg)


def main():
    sync = ServoSync()
    sync.open()

    connected = sync.scan()
    print(f"接続されているサーボID: {sorted(connected)}")
    if not connected:
        print("サーボが見つかりませんでした。配線・電源・COMポート設定を確認してください。")
        print("シミュレーターのみ起動します(実機への送信は行われません)。")

    print("--- SO-101 IK シミュレーター (実機同期版) ---")
    print("スライダーを動かすと、接続されているサーボにも角度が送信されます。")
    print("ウィンドウを閉じると終了します。")

    SyncIKSimulator(sync)
    try:
        plt.show()
    finally:
        sync.close()


if __name__ == "__main__":
    main()
