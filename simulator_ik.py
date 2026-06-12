"""SO-101アーム 3Dシミュレーター (逆運動学・スライダー操作版, PyVista)。

スライダーで手先位置(X, Y, Z)と手先のピッチ角を指定すると、数値IK
(ヤコビアン+反復法)でshoulder_pan, shoulder_lift, elbow_flex, wrist_flex
の4関節角度を求めてアームを動かす。wrist_roll(手首回転)とgripper(開閉)
は別スライダーで直接指定する。

IK計算はバックグラウンドスレッド(ArmWorker)で行い、描画は一定周期の
タイマーで更新する。スライダーのコールバックは目標値の更新のみを行う
軽量な処理のため、3Dビューのドラッグ回転・スクロールズームなどの
GUI操作がブロックされない。

使い方:
    python simulator_ik.py
"""

import time

import numpy as np
import pyvista as pv

from so101_kinematics import JOINT_NAMES, clamp_angles
from so101_ik import IK_JOINT_NAMES, end_effector_pose
from so101_view import ArmScene, PLOT_RANGE_XY, PLOT_Z_MIN, PLOT_Z_MAX
from arm_worker import ArmWorker
from home_position import load_home_pose
from calibration import calibrated_joint_limits_deg

# スライダーの範囲
X_RANGE = (-PLOT_RANGE_XY, PLOT_RANGE_XY)
Y_RANGE = (-PLOT_RANGE_XY, PLOT_RANGE_XY)
Z_RANGE = (PLOT_Z_MIN, PLOT_Z_MAX)
PITCH_RANGE_DEG = (-90.0, 90.0)

# 描画更新の周期 [秒]
REDRAW_INTERVAL = 0.05

# スライダーの並べ方 (画面右側に縦に並べる)
SLIDER_TOP = 0.95
SLIDER_SPACING = 0.13
SLIDER_POINT_A_X = 0.70
SLIDER_POINT_B_X = 0.98
SLIDER_TITLE_HEIGHT = 0.018


class IKSimulator:
    def __init__(self, servo_sync=None):
        # キャリブレーションのposition_min/maxから、有効な可動範囲[deg]を求める
        self.limits = calibrated_joint_limits_deg()

        # IK対象の4関節とwrist_roll/gripperの初期値 (ホームポジション)
        home_pose = load_home_pose()
        clamped = clamp_angles([home_pose[name] for name in JOINT_NAMES], self.limits)
        q4_init = np.array(clamped[:4])
        self.wrist_roll_init = clamped[4]
        self.gripper_init = clamped[5]

        # ホームポジションの手先位置・ピッチ角をスライダーの初期値にする
        x, y, z, pitch_rad = end_effector_pose(q4_init, self.wrist_roll_init, self.gripper_init)
        self.init_xyz_pitch = (x, y, z, np.degrees(pitch_rad))

        self.worker = ArmWorker(q4_init, self.wrist_roll_init, self.gripper_init,
                                 servo_sync=servo_sync, limits=self.limits)

        self.pl = pv.Plotter(title="SO-101 IK Simulator")
        self.scene = ArmScene(self.pl)

        self._build_sliders()

        self._redraw_from_state()

    def _add_slider(self, index, key, title, vmin, vmax, vinit):
        y = SLIDER_TOP - index * SLIDER_SPACING
        self.pl.add_slider_widget(
            callback=lambda value, k=key: self._on_slider_changed(k, value),
            rng=(vmin, vmax),
            value=vinit,
            title=title,
            pointa=(SLIDER_POINT_A_X, y),
            pointb=(SLIDER_POINT_B_X, y),
            interaction_event="always",
            style="modern",
            title_height=SLIDER_TITLE_HEIGHT,
            fmt="%.2f",
        )

    def _build_sliders(self):
        x, y, z, pitch_deg = self.init_xyz_pitch
        wr_lo, wr_hi = self.limits["wrist_roll"]
        gr_lo, gr_hi = self.limits["gripper"]

        # add_slider_widgetは作成時にもコールバックを呼ぶため、最初の
        # スライダー作成時点で全キーの初期値を参照できるようにしておく。
        self.values = {
            "x": x, "y": y, "z": z, "pitch": pitch_deg,
            "wrist_roll": self.wrist_roll_init, "gripper": self.gripper_init,
        }

        self._add_slider(0, "x", "X [m]", *X_RANGE, x)
        self._add_slider(1, "y", "Y [m]", *Y_RANGE, y)
        self._add_slider(2, "z", "Z [m]", *Z_RANGE, z)
        self._add_slider(3, "pitch", "Pitch [deg]", *PITCH_RANGE_DEG, pitch_deg)
        self._add_slider(4, "wrist_roll", "wrist_roll [deg]", wr_lo, wr_hi, self.wrist_roll_init)
        self._add_slider(5, "gripper", "gripper [deg]", gr_lo, gr_hi, self.gripper_init)

    def _on_slider_changed(self, key, value):
        """スライダー変更時のコールバック。重い処理は行わずワーカーに委譲する。"""
        self.values[key] = value
        target = np.array([
            self.values["x"],
            self.values["y"],
            self.values["z"],
            np.radians(self.values["pitch"]),
        ])
        self.worker.set_target(target, self.values["wrist_roll"], self.values["gripper"])

    def _redraw_from_state(self):
        q4, wrist_roll, gripper, tcp_pose = self.worker.get_state()
        angles = list(q4) + [wrist_roll, gripper]
        x, y, z, pitch_rad = tcp_pose

        joint_str = ", ".join(
            f"{name}={value:.1f}" for name, value in zip(IK_JOINT_NAMES, q4)
        )
        title = (
            "SO-101 IK simulator\n"
            f"TCP: x={x:.3f} y={y:.3f} z={z:.3f} pitch={np.degrees(pitch_rad):.1f}deg\n"
            f"{joint_str}"
        )
        self.scene.update(angles, title=title)

    def run(self):
        """ウィンドウを開き、閉じられる(右上の x ボタン)まで一定周期で再描画し続ける。"""
        self.pl.show(auto_close=False, interactive_update=True)
        while self.pl.iren is not None and not self.pl.iren.interactor.GetDone():
            self._redraw_from_state()
            try:
                self.pl.update()
            except Exception:
                break
            time.sleep(REDRAW_INTERVAL)
        if self.pl.iren is not None:
            self.pl.close()


def main():
    print("--- SO-101 IK シミュレーター ---")
    print("スライダーで手先位置(X, Y, Z)とピッチ角を指定すると、IKでアームが追従します。")
    print("wrist_roll(手首回転)とgripper(開閉)は直接スライダーで操作します。")
    print("ウィンドウを閉じると終了します。")

    sim = IKSimulator()
    try:
        sim.run()
    finally:
        sim.worker.stop()


if __name__ == "__main__":
    main()
