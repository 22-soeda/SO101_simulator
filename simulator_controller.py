"""SO-101アーム コントローラー操作版シミュレーター (実機サーボ同期)。

simulator_ik_sync.py と同じく3Dビュー・IK・実機サーボ同期を行うが、
手先位置(X, Y, Z)・ピッチ角・wrist_roll・gripperの操作方法が異なる。

- 画面左に並んだ6本のスライダーで操作する。各スライダーはゲーム
  コントローラーのスティックのように、中央(0)から左右にドラッグして
  いる間その方向に値が変化し続け、指を離すと中央に自動で戻って停止する。
- X/Y/Z/Pitchの変化量は、その時点の姿勢でのヤコビアン(微分IK)により
  関節角度の変化量に変換する。可動範囲外などでこれ以上目標方向に動けない
  場合、その変更は取り消される。シミュレーターのアームはその位置に
  移動せず、実機サーボへも指令は送信されない。

事前に calibrate_homing.py でキャリブレーション・ホームポジションを
設定しておくこと。

使い方:
    python simulator_controller.py
"""

import time

import numpy as np
import pyvista as pv

from calibration import load_calibration, calibrated_joint_limits_deg
from servo_sync import ServoSync
from so101_ik import IK_JOINT_NAMES, end_effector_pose, numerical_jacobian
from so101_kinematics import JOINT_NAMES, clamp_angles
from so101_view import ArmScene
from home_position import load_home_pose

# 描画・ジョグ操作の更新周期 [秒]
REDRAW_INTERVAL = 0.05

# ジョグ操作で1tickあたり変化する量
JOG_STEP_XYZ_M = 0.002   # X/Y/Z [m]
JOG_STEP_DEG = 0.5       # Pitch/wrist_roll/gripper [deg]

# X/Y/Zの目標変化量を関節角度の変化量に変換する疑似逆行列の特異値カットオフ
PINV_RCOND = 1e-3

# 1tickあたりの関節角度変化量の上限[deg] (特異姿勢付近での発散を防ぐ)
MAX_JOINT_STEP_DEG = 2.0

# ジョグスライダーの並べ方 (画面左側に縦に並べる)
JOG_SLIDER_TOP = 0.95
JOG_SLIDER_SPACING = 0.13
JOG_SLIDER_POINT_A_X = 0.02
JOG_SLIDER_POINT_B_X = 0.30
JOG_SLIDER_TITLE_HEIGHT = 0.018

# ジョグスライダーの値の範囲。中央(0)から離した方向に値が変化し続け、
# 指を離すと中央に戻る(デッドゾーンより内側では変化しない)。
JOG_SLIDER_RANGE = (-1.0, 1.0)
JOG_SLIDER_DEADZONE = 0.2

JOG_PARAMS = ["x", "y", "z", "pitch", "wrist_roll", "gripper"]
JOG_LABELS = {
    "x": "X", "y": "Y", "z": "Z",
    "pitch": "Pitch", "wrist_roll": "wrist_roll", "gripper": "gripper",
}


class ControllerSimulator:
    def __init__(self, calib, servo_sync=None):
        self.sync = servo_sync
        self.limits = calibrated_joint_limits_deg(calib)

        home_pose = load_home_pose()
        home_angles = clamp_angles([home_pose[name] for name in JOINT_NAMES], self.limits)
        self.q4 = np.array(home_angles[:4])
        self.wrist_roll = home_angles[4]
        self.gripper = home_angles[5]

        x, y, z, pitch_rad = end_effector_pose(self.q4, self.wrist_roll, self.gripper)
        self.values = {
            "x": x, "y": y, "z": z, "pitch": np.degrees(pitch_rad),
            "wrist_roll": self.wrist_roll, "gripper": self.gripper,
        }
        self.ik_ok = True

        self.jog_plus = {name: False for name in JOG_PARAMS}
        self.jog_minus = {name: False for name in JOG_PARAMS}

        self.pl = pv.Plotter(title="SO-101 Controller")
        self.scene = ArmScene(self.pl)
        self._build_ui()
        self._send_to_servo()
        self._redraw()

    def _build_ui(self):
        for index, name in enumerate(JOG_PARAMS):
            self._add_jog_slider(index, name)

        legend = (
            "ジョグスライダー (上から順):\n"
            + "\n".join(JOG_LABELS[name] for name in JOG_PARAMS)
            + "\n\n"
            "中央から左右にドラッグしている間、\n"
            "その方向に値が変化し続ける\n"
            "(ゲームコントローラーのスティックに\n"
            "相当)。指を離すと中央に戻って停止。\n\n"
            "X/Y/Z/Pitchはヤコビアンで関節角度に\n"
            "変換される。可動範囲外などでこれ以上\n"
            "動けない場合、変化は取り消され、\n"
            "アームは動かず実機にも指令を\n"
            "送信しない。"
        )
        self.pl.add_text(legend, position="upper_right", font_size=9, color="black")

    def _add_jog_slider(self, index, name):
        y = JOG_SLIDER_TOP - index * JOG_SLIDER_SPACING

        def on_change(value, name=name):
            self.jog_plus[name] = value > JOG_SLIDER_DEADZONE
            self.jog_minus[name] = value < -JOG_SLIDER_DEADZONE

        widget = self.pl.add_slider_widget(
            callback=on_change,
            rng=JOG_SLIDER_RANGE,
            value=0.0,
            title=JOG_LABELS[name],
            pointa=(JOG_SLIDER_POINT_A_X, y),
            pointb=(JOG_SLIDER_POINT_B_X, y),
            interaction_event="always",
            style="modern",
            title_height=JOG_SLIDER_TITLE_HEIGHT,
            fmt="%.1f",
        )

        def on_release(widget, _event, name=name):
            widget.GetRepresentation().SetValue(0.0)
            self.jog_plus[name] = False
            self.jog_minus[name] = False
            self.pl.render()

        widget.AddObserver("EndInteractionEvent", on_release)

    def _clamp_value(self, name, value):
        lo, hi = self.limits[name]
        return min(max(value, lo), hi)

    def _send_to_servo(self):
        if self.sync is None:
            return
        angles_deg = {
            "shoulder_pan": self.q4[0],
            "shoulder_lift": self.q4[1],
            "elbow_flex": self.q4[2],
            "wrist_flex": self.q4[3],
            "wrist_roll": self.wrist_roll,
            "gripper": self.gripper,
        }
        self.sync.send_angles(angles_deg)

    def _tick(self):
        """ジョグスライダーの状態に応じて目標値を更新し、反映する。"""
        deltas = {}
        for name in JOG_PARAMS:
            step = JOG_STEP_XYZ_M if name in ("x", "y", "z") else JOG_STEP_DEG
            d = 0.0
            if self.jog_plus[name]:
                d += step
            if self.jog_minus[name]:
                d -= step
            if d:
                deltas[name] = d

        if not deltas:
            return

        candidate = dict(self.values)
        candidate["wrist_roll"] = self._clamp_value(
            "wrist_roll", candidate["wrist_roll"] + deltas.get("wrist_roll", 0.0))
        candidate["gripper"] = self._clamp_value(
            "gripper", candidate["gripper"] + deltas.get("gripper", 0.0))
        wrist_roll = candidate["wrist_roll"]
        gripper = candidate["gripper"]

        xyz_pitch_changed = any(name in deltas for name in ("x", "y", "z", "pitch"))

        if xyz_pitch_changed:
            target_delta = np.array([
                deltas.get("x", 0.0),
                deltas.get("y", 0.0),
                deltas.get("z", 0.0),
                np.radians(deltas.get("pitch", 0.0)),
            ])

            # その時点の姿勢でのヤコビアンにより、目標変化量を1tick分の
            # 関節角度変化量に変換する(微分IK)。
            J, _ = numerical_jacobian(self.q4, wrist_roll, gripper)
            delta_q_deg = np.degrees(np.linalg.pinv(J, rcond=PINV_RCOND) @ target_delta)
            step_size = np.max(np.abs(delta_q_deg))
            if np.isfinite(step_size) and step_size > MAX_JOINT_STEP_DEG:
                delta_q_deg *= MAX_JOINT_STEP_DEG / step_size

            q4_new = np.array(clamp_angles(self.q4 + delta_q_deg, self.limits))
            if not np.isfinite(step_size) or np.allclose(q4_new, self.q4, atol=1e-9):
                # ヤコビアンが応答しない、または可動範囲外でこれ以上
                # 目標方向に動けない: 値・アーム・サーボへの指令は
                # すべて変更しない
                self.ik_ok = False
                return

            # 実際に到達した手先位置・姿勢をvaluesに反映する
            achieved = end_effector_pose(q4_new, wrist_roll, gripper)
            candidate["x"], candidate["y"], candidate["z"] = achieved[:3]
            candidate["pitch"] = np.degrees(achieved[3])
        else:
            q4_new = self.q4

        self.ik_ok = True
        self.q4 = q4_new
        self.values = candidate
        self.wrist_roll = wrist_roll
        self.gripper = gripper
        self._send_to_servo()

    def _redraw(self):
        angles = list(self.q4) + [self.wrist_roll, self.gripper]
        joint_str = ", ".join(
            f"{name}={value:.1f}" for name, value in zip(IK_JOINT_NAMES, self.q4)
        )
        ik_str = "OK" if self.ik_ok else "到達不可 (変化なし)"
        title = (
            "SO-101 Controller\n"
            f"X={self.values['x']:.3f} Y={self.values['y']:.3f} Z={self.values['z']:.3f} "
            f"Pitch={self.values['pitch']:.1f}deg  IK: {ik_str}\n"
            f"wrist_roll={self.wrist_roll:.1f}deg gripper={self.gripper:.1f}deg\n"
            f"{joint_str}"
        )
        self.scene.update(angles, title=title)

    def run(self):
        """ウィンドウを開き、閉じられる(右上の x ボタン)まで一定周期で更新し続ける。"""
        self.pl.show(auto_close=False, interactive_update=True)
        while self.pl.iren is not None and not self.pl.iren.interactor.GetDone():
            self._tick()
            self._redraw()
            try:
                self.pl.update()
            except Exception:
                break
            time.sleep(REDRAW_INTERVAL)
        if self.pl.iren is not None:
            self.pl.close()


def main():
    calib = load_calibration()
    sync = ServoSync(calibration=calib)
    sync.open()

    connected = sync.scan()
    print(f"接続されているサーボID: {sorted(connected)}")
    if not connected:
        print("サーボが見つかりませんでした。配線・電源・COMポート設定を確認してください。")
        print("シミュレーターのみ起動します(実機への送信は行われません)。")

    print("--- SO-101 コントローラー操作版シミュレーター ---")
    print("画面左のスライダーで手先位置・姿勢をジョグ操作します。")
    print("中央から離している間その方向に変化し、離すと中央に戻って停止します。")
    print("可動範囲外などでこれ以上動けない方向には変化しません。")
    print("ウィンドウを閉じると終了します。")

    sim = ControllerSimulator(calib, servo_sync=sync)
    try:
        sim.run()
    finally:
        sync.close()


if __name__ == "__main__":
    main()
