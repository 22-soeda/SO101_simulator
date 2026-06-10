"""SO-101アームの3D描画 (PyVista版)。

PyVistaはVTKベースの3Dビューアで、マウスドラッグでの視点回転や
スクロールでのズームなど、標準のトラックボール操作が最初から
直感的に動作する。`ArmScene`は描画オブジェクトを一度だけ作成し、
以降は各メッシュの`points`を更新するだけにすることで、再生成に
よるカメラ位置のリセットを避けている。
"""

import numpy as np
import pyvista as pv

from so101_kinematics import JOINT_NAMES, forward_kinematics

# 表示範囲 [m] (アームの最大リーチに合わせて調整)
PLOT_RANGE_XY = 0.45
PLOT_Z_MIN = -0.25
PLOT_Z_MAX = 0.55

AXIS_COLORS = ("red", "green", "blue")
AXIS_LENGTH = 0.04
BASE_RADIUS = 0.05


def _polyline_cells(n, closed=False):
    """PolyDataの`lines`配列 (vtk形式: [点数, idx0, idx1, ...]) を作る。"""
    idx = list(range(n))
    if closed:
        idx = idx + [0]
    return np.hstack([[len(idx)], idx])


class ArmScene:
    """SO-101アームの描画オブジェクトを保持し、データのみ更新する。"""

    def __init__(self, plotter):
        self.pl = plotter

        # アームのリンク (関節を結ぶポリライン)
        self.arm_mesh = pv.PolyData(np.zeros((7, 3)), lines=_polyline_cells(7))
        self.pl.add_mesh(self.arm_mesh, color="dodgerblue", line_width=5,
                         render_lines_as_tubes=True)

        # 関節点 (球で表示)
        self.joint_mesh = pv.PolyData(np.zeros((7, 3)))
        self.pl.add_mesh(self.joint_mesh, color="dodgerblue",
                         render_points_as_spheres=True, point_size=14)

        # グリッパー指 (2本)
        finger_lines = np.hstack([[2, 0, 1], [2, 2, 3]])
        self.finger_mesh = pv.PolyData(np.zeros((4, 3)), lines=finger_lines)
        self.pl.add_mesh(self.finger_mesh, color="red", line_width=5,
                         render_lines_as_tubes=True)

        # 手先座標系 (X:赤 Y:緑 Z:青)
        self.axis_meshes = []
        for color in AXIS_COLORS:
            mesh = pv.PolyData(np.zeros((2, 3)), lines=_polyline_cells(2))
            self.pl.add_mesh(mesh, color=color, line_width=4,
                             render_lines_as_tubes=True)
            self.axis_meshes.append(mesh)

        # ベース(床)の目安円 (姿勢に依存しないため一度だけ描画)
        theta = np.linspace(0, 2 * np.pi, 50, endpoint=False)
        circle_points = np.column_stack([
            BASE_RADIUS * np.cos(theta),
            BASE_RADIUS * np.sin(theta),
            np.zeros_like(theta),
        ])
        circle_mesh = pv.PolyData(circle_points, lines=_polyline_cells(len(theta), closed=True))
        self.pl.add_mesh(circle_mesh, color="gray", line_width=2)

        self.title_actor = self.pl.add_text("", position="upper_left", font_size=10)

        self.pl.show_grid(
            xtitle="X [m]", ytitle="Y [m]", ztitle="Z [m]",
            bounds=(-PLOT_RANGE_XY, PLOT_RANGE_XY,
                    -PLOT_RANGE_XY, PLOT_RANGE_XY,
                    PLOT_Z_MIN, PLOT_Z_MAX),
        )
        self.pl.camera.up = (0.0, 0.0, 1.0)
        self.pl.camera_position = [
            (1.2, -1.2, 0.9),
            (0.0, 0.0, 0.15),
            (0.0, 0.0, 1.0),
        ]

    def update(self, angles_deg, title=None):
        """6軸の角度[deg]に基づき、既存の描画オブジェクトのデータを更新する。"""
        points, end_effector_T, fingers = forward_kinematics(angles_deg)

        self.arm_mesh.points = points
        self.joint_mesh.points = points

        finger_points = np.vstack([
            fingers[0][0], fingers[0][1],
            fingers[1][0], fingers[1][1],
        ])
        self.finger_mesh.points = finger_points

        origin = end_effector_T[:3, 3]
        for i, mesh in enumerate(self.axis_meshes):
            direction = end_effector_T[:3, i] * AXIS_LENGTH
            mesh.points = np.vstack([origin, origin + direction])

        if title is None:
            joint_str = ", ".join(
                f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, angles_deg)
            )
            title = f"SO-101 joint angles [deg]\n{joint_str}"
        self.title_actor.set_text("upper_left", title)

        self.pl.render()
