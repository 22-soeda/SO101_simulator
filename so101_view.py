"""SO-101アームの3D描画の共通処理。

GUI操作(3Dビューのドラッグ回転など)中にフリーズしないよう、毎フレーム
`ax.cla()` で軸を作り直すのではなく、`ArmArtists` で一度だけ作成した
線オブジェクトのデータを更新する方式にしている。`cla()` は表示範囲や
カメラ視点(azim/elev)もリセットしてしまうため、ドラッグ中に呼ぶと
カクつきや操作不能の原因になる。
"""

import numpy as np

from so101_kinematics import JOINT_NAMES, forward_kinematics

# 軸方向の表示範囲 [m] (アームの最大リーチに合わせて調整)
PLOT_RANGE_XY = 0.45
PLOT_Z_MIN = -0.25
PLOT_Z_MAX = 0.55


def set_axes_equal(ax):
    """3D軸のスケールを等しくする。"""
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    centers = limits.mean(axis=1)
    radius = 0.5 * max(limits[:, 1] - limits[:, 0])
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


class ArmArtists:
    """SO-101アームの描画オブジェクトを保持し、データのみ更新する。"""

    AXIS_COLORS = ("r", "g", "b")
    AXIS_LENGTH = 0.04
    BASE_RADIUS = 0.05

    def __init__(self, ax):
        self.ax = ax

        (self.arm_line,) = ax.plot([], [], [], "-o", color="tab:blue",
                                    linewidth=3, markersize=6, label="arm links")

        self.finger_lines = [
            ax.plot([], [], [], color="tab:red", linewidth=3)[0]
            for _ in range(2)
        ]

        self.axis_lines = [
            ax.plot([], [], [], color=color, linewidth=2)[0]
            for color in self.AXIS_COLORS
        ]

        # ベース(床)の目安円。姿勢に依存しないため一度だけ描画する。
        theta = np.linspace(0, 2 * np.pi, 50)
        r = self.BASE_RADIUS
        ax.plot(r * np.cos(theta), r * np.sin(theta), np.zeros_like(theta),
                color="gray", linewidth=1)

        ax.set_xlim(-PLOT_RANGE_XY, PLOT_RANGE_XY)
        ax.set_ylim(-PLOT_RANGE_XY, PLOT_RANGE_XY)
        ax.set_zlim(PLOT_Z_MIN, PLOT_Z_MAX)
        set_axes_equal(ax)

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")

    def update(self, angles_deg, title=None):
        """6軸の角度[deg]に基づき、既存の描画オブジェクトのデータを更新する。"""
        points, end_effector_T, fingers = forward_kinematics(angles_deg)

        self.arm_line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])

        for line, (start, end) in zip(self.finger_lines, fingers):
            line.set_data_3d([start[0], end[0]], [start[1], end[1]], [start[2], end[2]])

        origin = end_effector_T[:3, 3]
        for i, line in enumerate(self.axis_lines):
            direction = end_effector_T[:3, i] * self.AXIS_LENGTH
            line.set_data_3d(
                [origin[0], origin[0] + direction[0]],
                [origin[1], origin[1] + direction[1]],
                [origin[2], origin[2] + direction[2]],
            )

        if title is None:
            joint_str = ", ".join(
                f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, angles_deg)
            )
            title = f"SO-101 joint angles [deg]\n{joint_str}"
        self.ax.set_title(title)


def draw_arm(ax, angles_deg, title=None):
    """3D軸 ax に一度だけアームを描画する(簡易用途向け)。

    繰り返し描画する場合は `ArmArtists` を使い、`cla()` を避けること。
    """
    ax.cla()
    artists = ArmArtists(ax)
    artists.update(angles_deg, title=title)
    return artists
