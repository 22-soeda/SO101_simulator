"""IK計算(と実機サーボ送信)をGUIスレッドから切り離して行うワーカー。

スライダー操作のコールバックは目標値を更新するだけにし、重い処理
(数値IKの反復計算や、シリアル通信を伴うサーボへの送信)はバック
グラウンドスレッドで行う。これにより、計算が多少時間がかかっても
matplotlibのGUIイベントループ(3Dビューのドラッグ回転など)が
ブロックされることがない。

ワーカーは常に「最新の目標値」だけを処理する。スライダーをドラッグ
して目標値が連続的に更新されても、処理が追いつかない場合は古い
目標値を読み飛ばし、最後に設定された値だけを計算する。
"""

import threading

import numpy as np

from so101_ik import end_effector_pose, solve_ik
from so101_kinematics import clamp_angles


class ArmWorker:
    def __init__(self, q4_init, wrist_roll_init, gripper_init, servo_sync=None, limits=None):
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)

        # ワーカースレッドへの入力 (最新の目標値)
        self._target = None
        self._wrist_roll = wrist_roll_init
        self._gripper = gripper_init
        self._target_dirty = False
        self._limits = limits

        # ワーカースレッドの出力 (最新の計算結果)
        self._q4 = np.array(q4_init, dtype=float)
        self._tcp_pose = end_effector_pose(self._q4, wrist_roll_init, gripper_init)

        self._servo_sync = servo_sync
        self._running = True

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_target(self, target_xyz_pitch, wrist_roll_deg, gripper_deg):
        """目標の手先位置・ピッチ角[rad]、wrist_roll/gripper[deg]を設定する(非ブロッキング)。"""
        with self._cond:
            self._target = np.array(target_xyz_pitch, dtype=float)
            self._wrist_roll = wrist_roll_deg
            self._gripper = gripper_deg
            self._target_dirty = True
            self._cond.notify()

    def get_state(self):
        """現在の関節角度・手先姿勢のスナップショットを返す(非ブロッキング)。"""
        with self._lock:
            return self._q4.copy(), self._wrist_roll, self._gripper, self._tcp_pose.copy()

    def stop(self):
        with self._cond:
            self._running = False
            self._cond.notify()
        self._thread.join(timeout=1.0)

    def _run(self):
        while True:
            with self._cond:
                while self._running and not self._target_dirty:
                    self._cond.wait()
                if not self._running:
                    return
                target = self._target
                wrist_roll = self._wrist_roll
                gripper = self._gripper
                q4_init = self._q4.copy()
                self._target_dirty = False

            q4 = solve_ik(target, q4_init, wrist_roll, gripper)
            q4 = np.array(clamp_angles(q4, self._limits))
            tcp_pose = end_effector_pose(q4, wrist_roll, gripper)

            with self._lock:
                self._q4 = q4
                self._tcp_pose = tcp_pose

            if self._servo_sync is not None:
                angles_deg = {
                    "shoulder_pan": q4[0],
                    "shoulder_lift": q4[1],
                    "elbow_flex": q4[2],
                    "wrist_flex": q4[3],
                    "wrist_roll": wrist_roll,
                    "gripper": gripper,
                }
                self._servo_sync.send_angles(angles_deg)
