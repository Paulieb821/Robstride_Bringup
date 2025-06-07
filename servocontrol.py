import serial
from time import sleep, time as _time

# ──────────────── Lobot low‑level helpers ─────────────────── #
LOBOT_FRAME_HEADER            = 0x55
LOBOT_CMD_SERVO_MOVE          = 3
LOBOT_CMD_GET_BATTERY_VOLTAGE = 15

def _u16(x):
    x = max(0, min(0xFFFF, int(x)))
    return list(x.to_bytes(2, "little"))

def _write_packet(handle, payload: bytearray):
    handle.write(bytearray((LOBOT_FRAME_HEADER, LOBOT_FRAME_HEADER)) + payload)

def _servo_move_packet(servo_id: int, pulse: int, duration_ms: int) -> bytearray:
    duration_ms = max(0, min(30000, duration_ms))
    pulse       = max(500, min(2500, pulse))
    return bytearray((
        8,
        LOBOT_CMD_SERVO_MOVE,
        1,
        *_u16(duration_ms),
        servo_id & 0xFF,
        *_u16(pulse)
    ))

# ────────────────── High‑level GripperController ────────────────── #
class GripperController:
    """
    Controls one or more Lobot PWM servos as a gripper.
    open_step()/close_step() move by `step_us` per call.
    toggle() snaps fully open ↔ closed.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 9600,
        servo_ids        = (1,),         # tuple of channel IDs
        pulse_open       = 500,          # µs for “open” position
        pulse_closed     = 2500,         # µs for “closed” position
        step_us          = 50            # increment per open_step/close_step
    ):
        # serial handle
        self.serial  = serial.Serial(port, baud, timeout=0)
        self.ids     = tuple(servo_ids)

        # allow scalars or per‑servo lists:
        if isinstance(pulse_open, int):
            self.p_open = [pulse_open] * len(self.ids)
        else:
            self.p_open = list(pulse_open)

        if isinstance(pulse_closed, int):
            self.p_closed = [pulse_closed] * len(self.ids)
        else:
            self.p_closed = list(pulse_closed)

        self.step    = abs(step_us)
        self._pos_us = self.p_open.copy()

        # communication check
        if not self._check_comm():
            # raise RuntimeError("Servo did not respond – check wiring/power")
            print("This errored fix later")

        # drive to known open
        self.open(duration_ms=300)

    def _check_comm(self) -> bool:
        """ Ask for battery voltage; expect ≥5 bytes back within 50 ms. """
        _write_packet(self.serial, bytearray((2, LOBOT_CMD_GET_BATTERY_VOLTAGE)))
        t0 = _time()
        while _time() - t0 < 0.05:
            if self.serial.in_waiting >= 5:
                self.serial.read(self.serial.in_waiting)
                return True
        return False

    def _move(self, pulses, duration_ms):
        if isinstance(pulses, int):
            pulses = [pulses] * len(self.ids)
        for sid, p in zip(self.ids, pulses):
            _write_packet(self.serial, _servo_move_packet(sid, p, duration_ms))
        self._pos_us = pulses.copy()

    def open(self,   duration_ms=300): self._move(self.p_open,   duration_ms)
    def close(self,  duration_ms=300): self._move(self.p_closed, duration_ms)

    def open_step(self,   duration_ms=40):
        new_pos = []
        for curr, tgt in zip(self._pos_us, self.p_open):
            # move “toward” tgt by at most self.step
            delta = max(-self.step, min(self.step, tgt - curr))
            new_pos.append(curr + delta)
        self._move(new_pos, duration_ms)

    def close_step(self,  duration_ms=40):
        new_pos = []
        for curr, tgt in zip(self._pos_us, self.p_closed):
            delta = max(-self.step, min(self.step, tgt - curr))
            new_pos.append(curr + delta)
        self._move(new_pos, duration_ms)

    def toggle(self):
        # pick whichever is farther from current
        if sum(abs(c - o) for c,o in zip(self._pos_us, self.p_open)) \
           < sum(abs(c - c_) for c,c_ in zip(self._pos_us, self.p_closed)):
            self.close()
        else:
            self.open()

    def __del__(self):
        try:
            if self.serial.is_open:
                self.serial.close()
        except:
            pass
