"""シミュレーターの関節角度を実機のFEETECHサーボへ反映するための処理。"""

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

import servo_config as config
from so101_kinematics import JOINT_LIMITS_DEG
from calibration import load_calibration


class ServoSync:
    """ポートの開閉、ID走査、角度<->サーボ位置変換、目標位置の書き込みを行う。"""

    def __init__(self, calibration=None):
        self.port_handler = PortHandler(config.DEVICE_NAME)
        self.packet_handler = PacketHandler(config.PROTOCOL_END)
        self.calibration = calibration if calibration is not None else load_calibration()
        self.connected_ids = set()

    def open(self):
        if not self.port_handler.openPort():
            raise IOError(f"ポート {config.DEVICE_NAME} を開けませんでした")
        if not self.port_handler.setBaudRate(config.BAUDRATE):
            raise IOError("ボーレートの設定に失敗しました")

    def close(self):
        self.port_handler.closePort()

    def scan(self):
        """JOINT_TO_SERVO_IDのうち、実際に接続されているサーボIDを調べる。"""
        self.connected_ids = set()
        for scs_id in config.JOINT_TO_SERVO_ID.values():
            _, comm_result, error = self.packet_handler.ping(self.port_handler, scs_id)
            if comm_result == COMM_SUCCESS and error == 0:
                self.connected_ids.add(scs_id)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, scs_id, config.ADDR_TORQUE_ENABLE, 1
                )
        return self.connected_ids

    def set_torque_enable(self, scs_id, enable):
        """サーボのトルクON/OFFを切り替える(OFFにすると手で動かせる)。"""
        self.packet_handler.write1ByteTxRx(
            self.port_handler, scs_id, config.ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def is_connected(self, joint_name):
        scs_id = config.JOINT_TO_SERVO_ID.get(joint_name)
        return scs_id in self.connected_ids

    def angle_to_position(self, joint_name, angle_deg):
        """論理角度[deg]を、可動範囲・キャリブレーションを反映したサーボ位置に変換する。"""
        calib = self.calibration[joint_name]

        lo, hi = JOINT_LIMITS_DEG[joint_name]
        angle_deg = min(max(angle_deg, lo), hi)

        steps_per_deg = config.STEPS_PER_REV / 360.0
        position = calib["home_position"] + calib["direction"] * angle_deg * steps_per_deg
        position = int(round(position))

        position = min(max(position, calib["position_min"]), calib["position_max"])
        position = min(max(position, config.POSITION_MIN), config.POSITION_MAX)
        return position

    def position_to_angle(self, joint_name, position):
        """サーボ位置を論理角度[deg]に変換する。

        サーボ位置は0〜4095(12bit)で一周するため、home_positionとの差分を
        -2048〜+2048ステップの範囲に正規化してから角度に変換する
        (0/4095の境界をまたぐ場合に角度が大きく飛ぶのを防ぐ)。
        """
        calib = self.calibration[joint_name]
        steps_per_deg = config.STEPS_PER_REV / 360.0

        diff = (position - calib["home_position"]) % config.STEPS_PER_REV
        if diff > config.STEPS_PER_REV / 2:
            diff -= config.STEPS_PER_REV

        return diff / calib["direction"] / steps_per_deg

    def read_position(self, scs_id):
        pos, comm_result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, scs_id, config.ADDR_PRESENT_POSITION
        )
        if comm_result != COMM_SUCCESS or error != 0:
            return None
        return pos

    def move_to_position(self, scs_id, position, speed=config.MOVING_SPEED, acc=config.MOVING_ACC):
        self.packet_handler.write1ByteTxRx(self.port_handler, scs_id, config.ADDR_GOAL_ACC, acc)
        self.packet_handler.write2ByteTxRx(self.port_handler, scs_id, config.ADDR_GOAL_SPEED, speed)
        self.packet_handler.write2ByteTxRx(self.port_handler, scs_id, config.ADDR_GOAL_POSITION, position)

    def send_angles(self, angles_deg):
        """{関節名: 論理角度[deg]} を、接続済みのサーボにのみ送信する。"""
        for joint_name, angle_deg in angles_deg.items():
            scs_id = config.JOINT_TO_SERVO_ID.get(joint_name)
            if scs_id is None or scs_id not in self.connected_ids:
                continue
            position = self.angle_to_position(joint_name, angle_deg)
            self.move_to_position(scs_id, position)
