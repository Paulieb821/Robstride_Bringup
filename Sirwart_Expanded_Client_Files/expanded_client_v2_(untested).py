# REPLACE DEFAULT SIRWART CLIENT.PY WITH THIS ONE
import dataclasses
import enum
import math
import struct
from typing import List

import can

class RunMode(enum.Enum):
    Operation = 0
    Position = 1
    Speed = 2
    Current = 3

class MotorMsg(enum.Enum):
    Info = 0
    Control = 1
    Feedback = 2
    Enable = 3
    Disable = 4
    ZeroPos = 6
    SetID = 7
    ReadParam = 17
    WriteParam = 18
    SaveFlash = 22

class MotorMode(enum.Enum):
    Reset = 0
    Calibration = 1
    Run = 2

class MotorError(enum.Enum):
    Undervoltage = 1
    Overcurrent = 2
    Overtemp = 4
    MagneticEncodingFault = 8
    HallEncodingFault = 16
    Uncalibrated = 32

@dataclasses.dataclass
class FeedbackResp:
    servo_id: int
    errors: List[MotorError]
    mode: MotorMode
    angle: float
    velocity: float
    torque: float
    temp: float

params = [
    ('run_mode', 0x7005),
    ('iq_ref', 0x7006),
    ('spd_ref', 0x700A),
    ('limit_torque', 0x700B),
    ('cur_kp', 0x7010),
    ('cur_ki', 0x7011),
    ('cur_fit_gain', 0x7014),
    ('loc_ref', 0x7016),
    ('limit_spd', 0x7017),
    ('limit_cur', 0x7018),
    ('mechpos', 0x7019),
    ('iqf', 0x701A),
    ('mechvel', 0x701B),
    ('vbus', 0x701C),
    ('loc_kp', 0x701E),
    ('spd_kp', 0x701F),
    ('spd_ki', 0x7020),
    ('spd_filt_gain', 0x7021),
    ('zero_sta', 0x7029)
]

param_ids_by_name = dict(params)
def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(min(x, x_max), x_min)
    return int((x - x_min) * ((1 << bits) - 1) / span)

class Client:
    def __init__(self, bus: can.BusABC, retry_count=2, recv_timeout=2, host_can_id=0xAA):
        self.bus = bus
        self.retry_count=retry_count
        self.recv_timeout = recv_timeout
        self.host_can_id = host_can_id
        self._recv_count = 0
        self._recv_error_count = 0

   

    def enable(self, motor_id: int, motor_model=1) -> FeedbackResp:
        self.bus.send(self._rs_msg(MotorMsg.Enable, self.host_can_id, motor_id, [0, 0, 0, 0, 0, 0, 0, 0]))
        resp = self._recv()
        return self._parse_feedback_resp(resp, motor_id, motor_model)

    def disable(self, motor_id: int, motor_model=1) -> FeedbackResp:
        self.bus.send(self._rs_msg(MotorMsg.Disable, self.host_can_id, motor_id, [0, 0, 0, 0, 0, 0, 0, 0]))
        resp = self._recv()
        return self._parse_feedback_resp(resp, motor_id, motor_model)

    def update_id(self, motor_id: int, new_motor_id: int):
        id_data_1 = self.host_can_id | (new_motor_id << 8)
        self.bus.send(self._rs_msg(MotorMsg.SetID, id_data_1, motor_id, [0, 0, 0, 0, 0, 0, 0, 0]))
        self._recv()

    def read_param(self, motor_id: int, param_id: int | str) -> float | RunMode:
        param_id = self._normalize_param_id(param_id)

        data = [param_id & 0xFF, param_id >> 8, 0, 0, 0, 0, 0, 0]
        self.bus.send(self._rs_msg(MotorMsg.ReadParam, self.host_can_id, motor_id, data))
        resp = self._recv()
        
        self._parse_and_validate_resp_arbitration_id(resp, MotorMsg.ReadParam.value, motor_id)

        resp_param_id = struct.unpack('<H', resp.data[:2])[0]
        if resp_param_id != param_id:
            raise Exception('Invalid param id')

        if param_id == 0x7005:
            value = RunMode(int(resp.data[4]))
        elif param_id == 0x7029:
            value = int(resp.data[4])
        else:
            value = struct.unpack('<f', resp.data[4:])[0]
        
        return value

    def write_param(self, motor_id: int, param_id: int | str, param_value: float | RunMode | int, motor_model=1) -> FeedbackResp:
        param_id = self._normalize_param_id(param_id)

        data = bytes([param_id & 0xFF, param_id >> 8, 0, 0])
        if param_id == 0x7005:
            if isinstance(param_value, RunMode):
                int_value = int(param_value.value)
            elif isinstance(param_value, int):
                int_value = param_value
            data += bytes([int_value, 0, 0, 0])
        elif param_id == 0x7029:
            if isinstance(param_value, int):
                int_value = param_value
            data += bytes([int_value, 0, 0, 0])
        else:
            data += struct.pack('<f', param_value)

        self.bus.send(self._rs_msg(MotorMsg.WriteParam, self.host_can_id, motor_id, data))
        resp = self._recv()

        return self._parse_feedback_resp(resp, motor_id, motor_model)

    def error_rate(self) -> float:
        return self._recv_error_count / self._recv_count
    
    def save_flash(self, motor_id: int, baud_code: int = 0x01):
        """
        Send the type‑22 frame so every parameter written with type‑18
        (e.g. zero_sta) is stored in flash.  baud_code selects the CAN
        baud‑rate to be used *after the next power‑cycle*.
            0x01 → 1 Mbit, 0x02 → 500 k, 0x03 → 250 k, 0x04 → 100 k
        """
        data = bytes([baud_code, 0, 0, 0, 0, 0, 0, 0])
        self.bus.send(self._rs_msg(MotorMsg.SaveFlash,
                                    self.host_can_id, motor_id, data))
        self._recv()        # waits for the ACK (Feedback frame)

    

    def zero_pos(self, motor_id: int, motor_model: int = 1) -> FeedbackResp:
        """
        1.  Physically move the joint to the angle you want to become 0 rad.
        2.  Call zero_pos().  The driver stores the current encoder count
            as its new mechanical zero.
        3.  (optional) call save_flash() so the offset survives power‑loss.
        """
        data = bytes([1, 0, 0, 0, 0, 0, 0, 0])      # Byte0 = 1, rest = 0
        self.bus.send(self._rs_msg(MotorMsg.ZeroPos,
                                   self.host_can_id,   # bit15‑8
                                   motor_id,           # bit7‑0
                                   data))
        resp = self._recv()                          # Feedback (=ACK)
        return self._parse_feedback_resp(resp, motor_id, motor_model)
    
    
    def control(self, motor_id: int, torque: float, mech_position: float, speed: float, kp: float, kd: float) -> FeedbackResp:
        T_MIN, T_MAX = -18.0, 18.0
        P_MIN, P_MAX = -4 * math.pi, 4 * math.pi
        V_MIN, V_MAX = -30.0, 30.0
        KP_MIN, KP_MAX = 0.0, 500.0
        KD_MIN, KD_MAX = 0.0, 5.0

        pos_uint = float_to_uint(mech_position, P_MIN, P_MAX, 16)
        spd_uint = float_to_uint(speed, V_MIN, V_MAX, 16)
        kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 16)
        kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 16)
        torque_uint = float_to_uint(torque, T_MIN, T_MAX, 16)

        data = [
            (pos_uint >> 8) & 0xFF, pos_uint & 0xFF,
            (spd_uint >> 8) & 0xFF, spd_uint & 0xFF,
            (kp_uint >> 8) & 0xFF, kp_uint & 0xFF,
            (kd_uint >> 8) & 0xFF, kd_uint & 0xFF,
        ]

        self.bus.send(self._rs_msg(MotorMsg.Control, torque_uint, motor_id, data))
        resp = self._recv()
        return self._parse_feedback_resp(resp, motor_id, motor_model=1)
    

    def _rs_msg(self, msg_type, id_data_1, id_data_2, data):
        arb_id = id_data_2 + (id_data_1 << 8) + (msg_type.value << 24)
        return can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)

    def _recv(self):
        retry_count = 0
        while retry_count <= self.retry_count:
            self._recv_count += 1
            resp = self.bus.recv(self.recv_timeout)
            if not resp:
                raise Exception('No response from motor received')
            if not resp.is_error_frame:
                return resp
            
            retry_count += 1
            self._recv_error_count += 1
            # TODO: make logging configurable
            print('received error:', resp)
        
        raise Exception('Error reading resp:', resp)

    def _parse_resp_abitration_id(self, aid):
        msg_type = (aid & 0x1F000000) >> 24
        msg_motor_id = (aid & 0xFF00) >> 8
        host_id = aid & 0xFF
        return msg_type, msg_motor_id, host_id

    def _parse_and_validate_resp_arbitration_id(self, resp, expected_msg_type, expected_motor_id):
        msg_type, msg_motor_id, host_id = self._parse_resp_abitration_id(resp.arbitration_id)
        if msg_type != expected_msg_type:
            raise Exception('Invalid msg_type', resp)
        if host_id != self.host_can_id:
            raise Exception('Invalid host CAN ID', resp)
        if msg_motor_id != expected_motor_id:
            raise Exception('Invalid motor ID received', resp)

        return msg_type, msg_motor_id, host_id

    def _parse_feedback_resp(self, resp, motor_id, motor_model):
        self._parse_and_validate_resp_arbitration_id(resp, MotorMsg.Feedback.value, motor_id)
 
        aid = resp.arbitration_id
        error_bits = (aid & 0x1F0000) >> 16
        errors = []
        for i in range(6):
            value = 1 << i
            if value & error_bits:
                errors.append(MotorError(value))

        mode = MotorMode((aid & 0x400000) >> 22)

        angle_raw = struct.unpack('>H', resp.data[0:2])[0]
        angle = (float(angle_raw) / 65535 * 8 * math.pi) - 4 * math.pi

        velocity_raw = struct.unpack('>H', resp.data[2:4])[0]
        velocity_range = 88 if motor_model == 1 else 30
        velocity = (float(velocity_raw) / 65535 * velocity_range) - velocity_range/2

        torque_raw = struct.unpack('>H', resp.data[4:6])[0]
        torque_range = 34 if motor_model == 1 else 240
        torque = (float(torque_raw) / 65535 * torque_range) - torque_range/2

        temp_raw = struct.unpack('>H', resp.data[6:8])[0]
        temp = float(temp_raw) / 10

        return FeedbackResp(motor_id, errors, mode, angle, velocity, torque, temp)

    def _normalize_param_id(self, param_id: int | str) -> int:
        if isinstance(param_id, str):
            return param_ids_by_name[param_id]
        
        return param_id