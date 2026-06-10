"""SO-101アーム 3Dシミュレーター (ターミナル入力版)。

ターミナルに6軸の角度[deg]を入力すると、3Dビューア上のアームが
その姿勢に動く。

使い方:
    python simulator.py

ターミナルでの入力例:
    0 0 0 0 0 0          -> 全軸を0度にする
    30 -20 40 0 90 45    -> 各軸を順に指定 (pan lift elbow wrist_flex wrist_roll gripper)
    help                 -> 使い方を再表示
    reset                -> 全軸を0度に戻す
    quit / exit          -> 終了
"""

import threading

import matplotlib.pyplot as plt
import numpy as np

from so101_kinematics import (
    JOINT_NAMES,
    JOINT_LIMITS_DEG,
    clamp_angles,
    forward_kinematics,
)

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


class Simulator:
    def __init__(self):
        self.angles = [0.0] * len(JOINT_NAMES)
        self.lock = threading.Lock()
        self.dirty = True
        self.running = True

        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection="3d")

    def set_angles(self, angles_deg):
        clamped = clamp_angles(angles_deg)
        with self.lock:
            self.angles = clamped
            self.dirty = True
        return clamped

    def draw(self):
        with self.lock:
            angles = list(self.angles)

        points, end_effector_T, fingers = forward_kinematics(angles)

        ax = self.ax
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

        title = ", ".join(
            f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, angles)
        )
        ax.set_title(f"SO-101 joint angles [deg]\n{title}")

        self.fig.canvas.draw_idle()

    def update_loop(self):
        while self.running:
            with self.lock:
                dirty = self.dirty
                self.dirty = False
            if dirty:
                self.draw()
            plt.pause(0.05)


def print_help():
    print("\n--- SO-101 シミュレーター 使い方 ---")
    print("6軸の角度[deg]をスペースまたはカンマ区切りで入力してください。")
    print("順番: " + " ".join(JOINT_NAMES))
    print("可動範囲:")
    for name in JOINT_NAMES:
        lo, hi = JOINT_LIMITS_DEG[name]
        print(f"  {name:14s}: {lo:.0f} ~ {hi:.0f}")
    print("例: 30 -20 40 0 90 45")
    print("コマンド: help / reset / quit\n")


def input_loop(sim: Simulator):
    print_help()
    while sim.running:
        try:
            line = input(">> ").strip()
        except EOFError:
            sim.running = False
            break

        if not line:
            continue

        cmd = line.lower()
        if cmd in ("quit", "exit", "q"):
            sim.running = False
            break
        if cmd == "help":
            print_help()
            continue
        if cmd == "reset":
            sim.set_angles([0.0] * len(JOINT_NAMES))
            print("全軸を0度にリセットしました。")
            continue

        tokens = line.replace(",", " ").split()
        if len(tokens) != len(JOINT_NAMES):
            print(f"エラー: {len(JOINT_NAMES)}個の数値を入力してください "
                  f"(入力されたのは{len(tokens)}個)。'help'で使い方を表示。")
            continue

        try:
            values = [float(t) for t in tokens]
        except ValueError:
            print("エラー: 数値として解釈できない値があります。")
            continue

        clamped = sim.set_angles(values)
        if clamped != values:
            print("一部の角度は可動範囲外のため制限されました。")
        print("更新: " + ", ".join(
            f"{name}={value:.1f}" for name, value in zip(JOINT_NAMES, clamped)
        ))


def main():
    sim = Simulator()
    sim.draw()

    thread = threading.Thread(target=input_loop, args=(sim,), daemon=True)
    thread.start()

    plt.show(block=False)
    sim.update_loop()


if __name__ == "__main__":
    main()
