"""SO-101アーム IKシミュレーター + 実機サーボ同期版。

simulator_ik.py のスライダー操作で計算された関節角度を、接続されている
FEETECHサーボへリアルタイムに送信する。未接続のサーボIDは無視されるため、
一部の関節のみ接続している場合でもそのまま使える。

IK計算とサーボへのシリアル通信はバックグラウンドスレッド(ArmWorker)で
行われるため、シリアル通信が多少遅延してもGUI操作(3Dビューのドラッグ
回転など)はブロックされない。

事前に calibrate_homing.py でキャリブレーション・ホームポジションを
設定しておくこと。

使い方:
    python simulator_ik_sync.py
"""

from simulator_ik import IKSimulator
from servo_sync import ServoSync


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

    sim = IKSimulator(servo_sync=sync)
    try:
        sim.run()
    finally:
        sim.worker.stop()
        sync.close()


if __name__ == "__main__":
    main()
