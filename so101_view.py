"""SO-101アームの3D描画の共通処理。"""

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


def draw_arm(ax, angles_deg, title=None):
    """6軸の角度[deg]からSO-101アームを3D軸 ax に描画する。"""
    points, end_effector_T, fingers = forward_kinematics(angles_deg)

    ax.cla()

    # アームのリンク (関節を結ぶ線)
    ax.plot(points[:, 0], points[:, 1], points[:, 2],
            "-o", color="tab:blue", linewidth=3, markersize=6,
            label="arm links")

    # グリッパー指
    for start, end in fingers:
        ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
                color="tab:red", linewidth=3)

    # 手先座標系 (X:赤 Y:緑 Z:青)
    origin = end_effector_T[:3, 3]
    axis_len = 0.04
    colors = ["r", "g", "b"]
    for i, color in enumerate(colors):
        direction = end_effector_T[:3, i] * axis_len
        ax.plot(
            [origin[0], origin[0] + direction[0]],
            [origin[1], origin[1] + direction[1]],
            [origin[2], origin[2] + direction[2]],
            color=color, linewidth=2,
        )

    # ベース (床) の目安円
    theta = np.linspace(0, 2 * np.pi, 50)
    r = 0.05
    ax.plot(r * np.cos(theta), r * np.sin(theta), np.zeros_like(theta),
            color="gray", linewidth=1)

    ax.set_xlim(-PLOT_RANGE_XY, PLOT_RANGE_XY)
    ax.set_ylim(-PLOT_RANGE_XY, PLOT_RANGE_XY)
    ax.set_zlim(PLOT_Z_MIN, PLOT_Z_MAX)
    set_axes_equal(ax)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    if title is None:
        title = ", ".join(
            f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, angles_deg)
        )
        title = f"SO-101 joint angles [deg]\n{title}"
    ax.set_title(title)
