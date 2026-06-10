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

import pyvista as pv

from so101_kinematics import (
    JOINT_NAMES,
    JOINT_LIMITS_DEG,
    clamp_angles,
)
from so101_view import ArmScene

# 入力された角度を反映する周期 [ms]
REDRAW_INTERVAL_MS = 50

# タイマーの最大実行回数 (REDRAW_INTERVAL_MSとの積が実用上の最大実行時間)
MAX_TIMER_STEPS = 10**8


class Simulator:
    def __init__(self):
        self.angles = [0.0] * len(JOINT_NAMES)
        self.lock = threading.Lock()
        self.dirty = True
        self.running = True

        self.pl = pv.Plotter(title="SO-101 Simulator")
        self.scene = ArmScene(self.pl)

    def set_angles(self, angles_deg):
        clamped = clamp_angles(angles_deg)
        with self.lock:
            self.angles = clamped
            self.dirty = True
        return clamped

    def draw(self):
        with self.lock:
            angles = list(self.angles)

        self.scene.update(angles)

    def _on_timer(self, _step):
        if not self.running:
            self.pl.close()
            return

        with self.lock:
            dirty = self.dirty
            self.dirty = False
        if dirty:
            self.draw()

    def start(self):
        self.draw()
        self.pl.add_timer_event(
            max_steps=MAX_TIMER_STEPS, duration=REDRAW_INTERVAL_MS, callback=self._on_timer
        )


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
    sim.start()

    thread = threading.Thread(target=input_loop, args=(sim,), daemon=True)
    thread.start()

    sim.pl.show()


if __name__ == "__main__":
    main()
