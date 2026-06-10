"""SO-101アーム 3Dシミュレーター (逆運動学・スライダー操作版)。

スライダーで手先位置(X, Y, Z)と手先のピッチ角を指定すると、数値IK
(ヤコビアン+反復法)でshoulder_pan, shoulder_lift, elbow_flex, wrist_flex
の4関節角度を求めてアームを動かす。wrist_roll(手首回転)とgripper(開閉)
は別スライダーで直接指定する。

使い方:
    python simulator_ik.py
"""

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from so101_kinematics import JOINT_LIMITS_DEG, forward_kinematics
from so101_ik import IK_JOINT_NAMES, end_effector_pose, solve_ik
from so101_view import draw_arm, PLOT_RANGE_XY, PLOT_Z_MIN, PLOT_Z_MAX

import numpy as np

# スライダーの範囲
X_RANGE = (-PLOT_RANGE_XY, PLOT_RANGE_XY)
Y_RANGE = (-PLOT_RANGE_XY, PLOT_RANGE_XY)
Z_RANGE = (PLOT_Z_MIN, PLOT_Z_MAX)
PITCH_RANGE_DEG = (-90.0, 90.0)


class IKSimulator:
    def __init__(self):
        # IK対象の4関節の初期値 (ゼロ姿勢)
        self.q4 = np.zeros(4)
        self.wrist_roll = 0.0
        self.gripper = 0.0

        # ゼロ姿勢の手先位置・ピッチ角をスライダーの初期値にする
        x, y, z, pitch_rad = end_effector_pose(self.q4, self.wrist_roll, self.gripper)
        self.init_xyz_pitch = (x, y, z, np.degrees(pitch_rad))

        self._updating = False

        self.fig = plt.figure(figsize=(8, 9))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.subplots_adjust(bottom=0.40)

        self.sliders = {}
        self._build_sliders()

        self.redraw()

    def _add_slider(self, index, label, vmin, vmax, vinit):
        ax_slider = self.fig.add_axes([0.25, 0.03 + 0.04 * index, 0.6, 0.03])
        slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit)
        slider.on_changed(self._on_slider_changed)
        self.sliders[label] = slider

    def _build_sliders(self):
        x, y, z, pitch_deg = self.init_xyz_pitch
        wr_lo, wr_hi = JOINT_LIMITS_DEG["wrist_roll"]
        gr_lo, gr_hi = JOINT_LIMITS_DEG["gripper"]

        # 下から順に並ぶので、上に表示したいものほどindexを大きくする
        self._add_slider(5, "X [m]", *X_RANGE, x)
        self._add_slider(4, "Y [m]", *Y_RANGE, y)
        self._add_slider(3, "Z [m]", *Z_RANGE, z)
        self._add_slider(2, "Pitch [deg]", *PITCH_RANGE_DEG, pitch_deg)
        self._add_slider(1, "wrist_roll [deg]", wr_lo, wr_hi, self.wrist_roll)
        self._add_slider(0, "gripper [deg]", gr_lo, gr_hi, self.gripper)

    def _on_slider_changed(self, _value):
        if self._updating:
            return

        target = np.array([
            self.sliders["X [m]"].val,
            self.sliders["Y [m]"].val,
            self.sliders["Z [m]"].val,
            np.radians(self.sliders["Pitch [deg]"].val),
        ])
        self.wrist_roll = self.sliders["wrist_roll [deg]"].val
        self.gripper = self.sliders["gripper [deg]"].val

        self.q4 = solve_ik(target, self.q4, self.wrist_roll, self.gripper)
        self.redraw()

    def redraw(self):
        angles = list(self.q4) + [self.wrist_roll, self.gripper]
        x, y, z, pitch_rad = end_effector_pose(self.q4, self.wrist_roll, self.gripper)

        joint_str = ", ".join(
            f"{name}={value:.1f}" for name, value in zip(IK_JOINT_NAMES, self.q4)
        )
        title = (
            "SO-101 IK simulator\n"
            f"TCP: x={x:.3f} y={y:.3f} z={z:.3f} pitch={np.degrees(pitch_rad):.1f}deg\n"
            f"{joint_str}"
        )
        draw_arm(self.ax, angles, title=title)
        self.fig.canvas.draw_idle()


def main():
    print("--- SO-101 IK シミュレーター ---")
    print("スライダーで手先位置(X, Y, Z)とピッチ角を指定すると、IKでアームが追従します。")
    print("wrist_roll(手首回転)とgripper(開閉)は直接スライダーで操作します。")
    print("ウィンドウを閉じると終了します。")

    sim = IKSimulator()
    plt.show()


if __name__ == "__main__":
    main()
