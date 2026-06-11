"""3Dシミュレーター画面を見ながら対話的にサーボのキャリブレーションを行う。

calibrate_servos.py(コマンドラインのみ)では回転方向の正負がわかりにくい
ため、3Dシミュレーターを表示しながら以下のように設定できるようにした版。

手順:
  1. このスクリプトを実行する。接続されているサーボのトルクをOFFにし、
     脱力状態のまま各関節の現在位置を読み取り続ける。
  2. アームを手で動かすと、3Dシミュレーター上の対応する軸もリアルタイムに
     回転する。
  3. 画面左側のチェックボックスは各関節の回転方向(上から
     shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll,
     gripperの順)。チェックを入り切りすると回転方向の符号が反転し、
     シミュレーター上の回転方向も反転するので、実機とシミュレーターの
     回転方向が一致するように調整する。
  4. 実機アームを、シミュレーター上の論理角度0度(初期姿勢)に
     対応する姿勢に手で動かす。
  5. 画面左下の「初期姿勢として記録」チェックボックスをクリックすると、
     その時点の各関節のサーボ位置が論理角度0度のオフセット
     (home_position)として記録され、回転方向とあわせて
     servo_calibration.json に保存される。記録直後は実機の角度が
     ほぼ0度になるため、シミュレーターのアームも初期姿勢(全軸0度)に
     近い姿勢で表示される。
  6. そのままアームを手で動かし、シミュレーターの起動姿勢にしたい
     ポーズにしてからウィンドウを閉じる。終了時点の各関節角度が
     ホームポジションとして home_position.json に保存され、
     simulator.py / simulator_ik.py の起動姿勢になる。
  7. 各関節を可動範囲の両端まで手で動かすと、画面に表示される
     range=[min, max] が更新されていく。一通り動かし終えたら
     画面左側の「可動範囲を記録」(青)チェックボックスをクリック
     すると、ここまでに観測した範囲が各関節の
     position_min/position_max として servo_calibration.json に
     保存される(ほとんど動かしていない関節の値は変更されない)。
     やり直したい場合は「範囲計測リセット」(橙)チェックボックスで
     計測値をリセットできる。

使い方:
    python calibrate_servos_visual.py
"""

import time

import pyvista as pv

import servo_config as config
from servo_sync import ServoSync
from calibration import load_calibration, save_calibration, CALIBRATION_FILE
from home_position import save_home_pose, HOME_POSITION_FILE
from so101_kinematics import JOINT_NAMES, JOINT_LIMITS_DEG, clamp_angles
from so101_view import ArmScene

REDRAW_INTERVAL = 0.05
STEPS_PER_DEG = config.STEPS_PER_REV / 360.0

# 可動範囲記録: この角度[deg]以上動かした関節のみ position_min/max を更新する
MIN_RANGE_DEG = 5.0

# チェックボックスの配置 (画面左側に縦に並べる, ピクセル座標)
CHECKBOX_X = 10
CHECKBOX_TOP_Y = 700
CHECKBOX_SPACING = 45
CHECKBOX_GAP = 20
CHECKBOX_SIZE = 35

JOINT_LABELS_JA = {
    "shoulder_pan": "1: shoulder_pan",
    "shoulder_lift": "2: shoulder_lift",
    "elbow_flex": "3: elbow_flex",
    "wrist_flex": "4: wrist_flex",
    "wrist_roll": "5: wrist_roll",
    "gripper": "6: gripper",
}


class CalibrationGUI:
    def __init__(self, sync, connected):
        self.sync = sync
        self.connected = connected
        self.calib = sync.calibration

        self.pl = pv.Plotter(title="SO-101 キャリブレーション (対話版)")
        self.scene = ArmScene(self.pl)

        self.last_angles = [0.0] * len(JOINT_NAMES)
        self.angle_min = {name: None for name in JOINT_NAMES}
        self.angle_max = {name: None for name in JOINT_NAMES}
        self.message = ""

        self._build_ui()

    def _build_ui(self):
        y = CHECKBOX_TOP_Y
        for name in JOINT_NAMES:
            def direction_callback(flag, name=name):
                self.calib[name]["direction"] = -1 if flag else 1

            initial_flag = (self.calib[name]["direction"] == -1)
            self.pl.add_checkbox_button_widget(
                direction_callback, value=initial_flag,
                position=(CHECKBOX_X, y), size=CHECKBOX_SIZE,
                color_on="red", color_off="lightgray",
            )
            y -= CHECKBOX_SPACING

        y -= CHECKBOX_GAP

        def capture_callback(_flag):
            self._capture_home_pose()

        self.pl.add_checkbox_button_widget(
            capture_callback, value=False,
            position=(CHECKBOX_X, y), size=CHECKBOX_SIZE,
            color_on="limegreen", color_off="limegreen",
        )
        y -= CHECKBOX_SPACING

        def record_range_callback(_flag):
            self._record_range()

        self.pl.add_checkbox_button_widget(
            record_range_callback, value=False,
            position=(CHECKBOX_X, y), size=CHECKBOX_SIZE,
            color_on="dodgerblue", color_off="dodgerblue",
        )
        y -= CHECKBOX_SPACING

        def reset_range_callback(flag):
            self._reset_range_tracking(flag)

        self.pl.add_checkbox_button_widget(
            reset_range_callback, value=False,
            position=(CHECKBOX_X, y), size=CHECKBOX_SIZE,
            color_on="orange", color_off="orange",
        )

        legend = (
            "チェックボックス (上から):\n"
            + "\n".join(JOINT_LABELS_JA[name] for name in JOINT_NAMES)
            + "\n  -> チェックで回転方向を反転\n"
            + "[緑] -> 現在の姿勢を初期姿勢(0度)\n"
            "       として記録\n"
            + "[青] -> ここまでの可動範囲を\n"
            "       position_min/maxとして記録\n"
            + "[橙] -> 可動範囲の計測をリセット"
        )
        self.pl.add_text(legend, position="upper_right", font_size=9, color="black")

        self.status_actor = self.pl.add_text("", position="lower_right", font_size=9, color="black")

    def _capture_home_pose(self):
        captured = []
        for name in JOINT_NAMES:
            scs_id = config.JOINT_TO_SERVO_ID[name]
            if scs_id not in self.connected:
                continue
            position = self.sync.read_position(scs_id)
            if position is None:
                continue

            entry = self.calib[name]
            direction = entry["direction"]
            home_position = int(round(position))

            lo, hi = JOINT_LIMITS_DEG[name]
            pos_at_lo = home_position + direction * lo * STEPS_PER_DEG
            pos_at_hi = home_position + direction * hi * STEPS_PER_DEG
            position_min = int(round(min(pos_at_lo, pos_at_hi)))
            position_max = int(round(max(pos_at_lo, pos_at_hi)))
            position_min = max(position_min, config.POSITION_MIN)
            position_max = min(position_max, config.POSITION_MAX)

            entry["home_position"] = home_position
            entry["position_min"] = position_min
            entry["position_max"] = position_max
            captured.append(name)

        save_calibration(self.calib)
        self._reset_range_tracking()
        self.message = f"記録: {', '.join(captured)}\n-> {CALIBRATION_FILE} に保存しました。"

    def _record_range(self):
        """ここまでに観測した角度の最小/最大を、各関節のposition_min/maxとして記録する。"""
        updated = []
        skipped = []
        for name in JOINT_NAMES:
            scs_id = config.JOINT_TO_SERVO_ID[name]
            if scs_id not in self.connected:
                continue

            amin, amax = self.angle_min[name], self.angle_max[name]
            if amin is None or amax is None or (amax - amin) < MIN_RANGE_DEG:
                skipped.append(name)
                continue

            entry = self.calib[name]
            direction = entry["direction"]
            home_position = entry["home_position"]

            pos_at_min = home_position + direction * amin * STEPS_PER_DEG
            pos_at_max = home_position + direction * amax * STEPS_PER_DEG
            position_min = int(round(min(pos_at_min, pos_at_max)))
            position_max = int(round(max(pos_at_min, pos_at_max)))
            position_min = max(position_min, config.POSITION_MIN)
            position_max = min(position_max, config.POSITION_MAX)

            entry["position_min"] = position_min
            entry["position_max"] = position_max
            updated.append(name)

        save_calibration(self.calib)

        lines = []
        if updated:
            lines.append(f"範囲記録: {', '.join(updated)}")
            lines.append(f"-> {CALIBRATION_FILE} に保存しました。")
        if skipped:
            lines.append(f"(動きが小さくスキップ: {', '.join(skipped)})")
        self.message = "\n".join(lines) if lines else "記録対象がありません。"

    def _reset_range_tracking(self, _flag=None):
        """可動範囲の計測(最小/最大角度)をリセットする。"""
        self.angle_min = {name: None for name in JOINT_NAMES}
        self.angle_max = {name: None for name in JOINT_NAMES}
        if _flag is not None:
            self.message = "可動範囲の計測をリセットしました。\n各関節を可動範囲の両端まで動かしてください。"

    def _read_angles(self):
        angles = list(self.last_angles)
        status_lines = []
        for i, name in enumerate(JOINT_NAMES):
            scs_id = config.JOINT_TO_SERVO_ID[name]
            if scs_id not in self.connected:
                status_lines.append(f"{name}: 未接続")
                angles[i] = 0.0
                continue

            position = self.sync.read_position(scs_id)
            if position is None:
                status_lines.append(f"{name}: 読み取り失敗")
                continue

            entry = self.calib[name]
            angle = self.sync.position_to_angle(name, position)
            angles[i] = angle

            if self.angle_min[name] is None or angle < self.angle_min[name]:
                self.angle_min[name] = angle
            if self.angle_max[name] is None or angle > self.angle_max[name]:
                self.angle_max[name] = angle

            status_lines.append(
                f"{name}: angle={angle:7.1f}deg dir={entry['direction']:+d} "
                f"pos={position:5d} home={entry['home_position']} "
                f"range=[{self.angle_min[name]:6.1f},{self.angle_max[name]:6.1f}]"
            )

        self.last_angles = angles
        return angles, status_lines

    def _update(self):
        angles, status_lines = self._read_angles()
        title = "SO-101 キャリブレーション (対話版)\n" + "\n".join(status_lines)
        self.scene.update(angles, title=title)
        if self.message:
            self.status_actor.set_text("lower_right", self.message)

    def run(self):
        self.pl.show(auto_close=False, interactive_update=True)
        while self.pl.iren is not None and not self.pl.iren.interactor.GetDone():
            self._update()
            try:
                self.pl.update()
            except Exception:
                break
            time.sleep(REDRAW_INTERVAL)
        if self.pl.iren is not None:
            self.pl.close()

        self._save_home_pose()

    def _save_home_pose(self):
        """終了時点の各関節角度をホームポジションとして保存する。"""
        clamped = clamp_angles(self.last_angles)
        home_pose = dict(zip(JOINT_NAMES, clamped))
        save_home_pose(home_pose)
        print(f"\n終了時の姿勢をホームポジションとして {HOME_POSITION_FILE} に保存しました。")
        for name in JOINT_NAMES:
            print(f"  {name}: {home_pose[name]:.1f} deg")


def main():
    calib = load_calibration()
    sync = ServoSync(calibration=calib)
    sync.open()

    connected = sync.scan()
    print(f"接続されているサーボID: {sorted(connected)}")
    if not connected:
        print("サーボが見つかりませんでした。配線・電源・COMポート設定を確認してください。")
        sync.close()
        return

    # 脱力状態にして、手でアームを動かせるようにする
    for name in JOINT_NAMES:
        scs_id = config.JOINT_TO_SERVO_ID[name]
        if scs_id in connected:
            sync.set_torque_enable(scs_id, False)

    print("サーボのトルクをOFFにしました。")
    print("アームを手で動かすと、3Dビュー上のアームも連動して動きます。")
    print("左側のチェックボックスで各関節の回転方向を調整し、")
    print("実機をシミュレーターの初期姿勢(全軸0度)に合わせてから")
    print("緑のチェックボックスをクリックすると、その姿勢を")
    print("論理角度0度として servo_calibration.json に保存します。")
    print()
    print("各関節を可動範囲の両端まで動かすと、画面のrange表示が更新されます。")
    print("青のチェックボックスをクリックすると、観測した範囲を")
    print("position_min/maxとして servo_calibration.json に保存します")
    print("(ほとんど動かしていない関節は変更されません)。")
    print("橙のチェックボックスをクリックすると、範囲の計測をやり直せます。")
    print()
    print("ウィンドウを閉じると、その時点の姿勢がホームポジションとして")
    print("home_position.json に保存されます。")

    gui = CalibrationGUI(sync, connected)
    try:
        gui.run()
    finally:
        sync.close()


if __name__ == "__main__":
    main()
