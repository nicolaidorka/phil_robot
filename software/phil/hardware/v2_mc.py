"""v2 Microcontroller driver for Phil after reflashing to octopi_firmware_v2.

This is the microstep-capable counterpart to :mod:`legacy_mc.py`. The octopi v2
firmware (``firmware/octopi_firmware_v2/main_controller_teensy41``) speaks a
WIDER protocol over USB serial, verified by reading the firmware source:

  * command frame  = 8 bytes:  [cmd_id, opcode, p0, p1, p2, p3, p4, crc8]
                     positions are a 4-byte (32-bit) signed BE value in p0..p3.
  * status packet  = 24 bytes: [cmd_id, status, X(4 BE), Y(4 BE), Z(4 BE),
                                 THETA(4 BE), ..., byte18=switches/joystick, ...]
  * CRC            = CRC-8/CCITT (poly 0x07), SAME table as legacy/crc8.cpp;
                     computed over the first CMD_LENGTH-1 bytes.
  * units          = THE WHOLE POINT: v2 commands AND reports in microsteps at
                     the firmware's configured microstepping. There is NO
                     full-step rounding (unlike legacy_mc, which divided by 8),
                     so the command grid is ~32x finer -> sub-mm at the arm tip.

Differences from the custom legacy firmware that drove the same board before:
  - legacy: 6-byte cmd / 20-byte status / 3-byte (24-bit) positions / commands
    in full-steps (the coarse grid we are removing).
  - v2:     8-byte cmd / 24-byte status / 4-byte positions / commands in microsteps.

Unlike the legacy firmware, v2 HONORS velocity/accel limits (set from def_phil.h
at startup), so motion is ramped by the TMC4361A rather than a fixed profile.

This class mirrors the subset of the Microcontroller interface that
:class:`phil.robot.PhilRobot` uses, with a background read thread that keeps
byte-alignment on the delimiter-less 24-byte stream (same approach as legacy).

NOTE: the phil package speaks "repo usteps". With v2 the natural unit IS the
firmware microstep, so this driver reports/commands microsteps 1:1 (``SCALE``).
The teach table / kinematics are RE-TAUGHT after the reflash in this finer unit;
the legacy calibration in config/pre-reflash-backup/ does NOT carry over.
"""
from __future__ import annotations

import threading
import time

import serial
import serial.tools.list_ports as lp

from .legacy_mc import crc8, find_controller   # identical CRC table + port discovery

CMD_LENGTH = 8
MSG_LENGTH = 24
N_BYTES_POS = 4
POS_OFFSETS = {"X": 2, "Y": 6, "Z": 10, "THETA": 14}
SWITCH_BYTE = 18

# v2 reports microsteps directly; keep a 1:1 boundary so joint counts == microsteps.
SCALE = 1

# opcodes (firmware main_controller_teensy41.ino, verified at e566e01^)
MOVE_X, MOVE_Y, MOVE_Z = 0, 1, 2
HOME_OR_ZERO = 5
MOVETO_X, MOVETO_Y, MOVETO_Z = 6, 7, 8
SET_AXIS_DISABLE_ENABLE = 32
INITIALIZE = 254
RESET = 255

# HOME_OR_ZERO payload[1] conventions (v2 def, NOTE: differ from legacy!)
HOME_POSITIVE, HOME_NEGATIVE, HOME_OR_ZERO_ZERO = 0, 1, 2
AXIS = {"X": 0, "Y": 1, "Z": 2}

# status byte[1] codes
COMPLETED, IN_PROGRESS, CHECKSUM_ERROR = 0, 1, 2


def _s32(value: int) -> bytes:
    """Signed 32-bit big-endian (the v2 position payload width)."""
    value = max(-2147483648, min(2147483647, int(value)))
    return (value & 0xFFFFFFFF).to_bytes(4, "big")


class V2Microcontroller:
    def __init__(self, version="Teensy", sn=None, port=None):
        self.port = port or find_controller(version, sn)
        if self.port is None:
            raise IOError("no Teensy/Arduino controller found")
        self.serial = serial.Serial(self.port, 2000000, timeout=1)
        time.sleep(0.2)

        self._cmd_id = 0
        self.mcu_cmd_execution_in_progress = False

        self.x_pos = self.y_pos = self.z_pos = self.theta_pos = 0
        self.button_and_switch_state = 0

        self._buf = bytearray()
        self._lock = threading.Lock()
        self._stop = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # --------------------------------------------------------------- reader
    def _read_loop(self):
        while not self._stop:
            try:
                chunk = self.serial.read(MSG_LENGTH)
            except Exception:
                break
            if not chunk:
                continue
            self._buf.extend(chunk)
            self._resync_and_parse()

    def _resync_and_parse(self):
        """Find a 24-byte boundary using the echoed cmd_id, then parse latest."""
        buf = self._buf
        if len(buf) < MSG_LENGTH * 2:
            return
        target = self._cmd_id
        best = None
        for off in range(MSG_LENGTH):
            if off + MSG_LENGTH * 2 > len(buf):
                break
            ids = [buf[off + k * MSG_LENGTH] for k in range(2)]
            sts = [buf[off + 1 + k * MSG_LENGTH] for k in range(2)]
            if all(s in (0, 1, 2, 3, 4) for s in sts):
                if all(i == target for i in ids):
                    best = off
                    break
                if best is None:
                    best = off
        if best is None:
            if len(buf) > MSG_LENGTH * 8:
                del buf[:-MSG_LENGTH * 4]
            return
        last = best + ((len(buf) - best) // MSG_LENGTH - 1) * MSG_LENGTH
        pk = bytes(buf[last:last + MSG_LENGTH])
        self._cmd_id_mcu = pk[0]
        self._cmd_execution_status = pk[1]
        ox, oy, oz, ot = (POS_OFFSETS[a] for a in ("X", "Y", "Z", "THETA"))
        self.x_pos = int.from_bytes(pk[ox:ox + 4], "big", signed=True)
        self.y_pos = int.from_bytes(pk[oy:oy + 4], "big", signed=True)
        self.z_pos = int.from_bytes(pk[oz:oz + 4], "big", signed=True)
        self.theta_pos = int.from_bytes(pk[ot:ot + 4], "big", signed=True)
        self.button_and_switch_state = pk[SWITCH_BYTE]
        if self.mcu_cmd_execution_in_progress and pk[0] == self._cmd_id and pk[1] == COMPLETED:
            self.mcu_cmd_execution_in_progress = False
        del buf[:last]

    # -------------------------------------------------------------- sending
    def _send(self, opcode, p0=0, p1=0, p2=0, p3=0, p4=0, expect_ack=True):
        self._cmd_id = (self._cmd_id + 1) % 256
        body = bytes([self._cmd_id, opcode,
                      p0 & 0xFF, p1 & 0xFF, p2 & 0xFF, p3 & 0xFF, p4 & 0xFF])
        frame = body + bytes([crc8(body)])
        if expect_ack:
            self.mcu_cmd_execution_in_progress = True
        self.serial.write(frame)

    def _send_pos(self, opcode, microsteps):
        # v2 commands directly in microsteps -- NO full-step rounding (the fix).
        b = _s32(int(round(microsteps / SCALE)))
        self._send(opcode, b[0], b[1], b[2], b[3])

    # ------------------------------------------------------------- motion
    def move_x_usteps(self, u): self._send_pos(MOVE_X, u)
    def move_y_usteps(self, u): self._send_pos(MOVE_Y, u)
    def move_z_usteps(self, u): self._send_pos(MOVE_Z, u)
    def move_x_to_usteps(self, u): self._send_pos(MOVETO_X, u)
    def move_y_to_usteps(self, u): self._send_pos(MOVETO_Y, u)
    def move_z_to_usteps(self, u): self._send_pos(MOVETO_Z, u)

    # home/zero: payload [axis, mode]; mode=HOME_OR_ZERO_ZERO sets current pos = 0
    def home_x(self): self._send(HOME_OR_ZERO, AXIS["X"], HOME_NEGATIVE)
    def home_y(self): self._send(HOME_OR_ZERO, AXIS["Y"], HOME_NEGATIVE)
    def home_z(self): self._send(HOME_OR_ZERO, AXIS["Z"], HOME_NEGATIVE)
    def home_xy(self):
        self.home_x(); self.wait_till_operation_is_completed(); self.home_y()
    def zero_x(self): self._send(HOME_OR_ZERO, AXIS["X"], HOME_OR_ZERO_ZERO)
    def zero_y(self): self._send(HOME_OR_ZERO, AXIS["Y"], HOME_OR_ZERO_ZERO)
    def zero_z(self): self._send(HOME_OR_ZERO, AXIS["Z"], HOME_OR_ZERO_ZERO)

    # ------------------------------------------------------------- status
    def get_pos(self):
        return (int(round(self.x_pos * SCALE)),
                int(round(self.y_pos * SCALE)),
                int(round(self.z_pos * SCALE)),
                self.theta_pos)

    def is_busy(self):
        return self.mcu_cmd_execution_in_progress

    def wait_till_operation_is_completed(self, timeout=5):
        t0 = time.time()
        while self.is_busy() and time.time() - t0 < timeout:
            time.sleep(0.01)
        self.mcu_cmd_execution_in_progress = False

    # ----------------------------------------------------- lifecycle/config
    def reset(self):
        self._send(RESET, expect_ack=False)

    def initialize_drivers(self):
        self._send(INITIALIZE, expect_ack=False)

    def configure_actuators(self):
        # v2 applies def_phil.h driver config (current, microstepping, vel/accel)
        # at INITIALIZE. The per-field CONFIGURE_* opcodes are not needed for
        # bring-up; rely on the firmware defaults like the legacy driver did.
        pass

    def set_max_velocity_acceleration(self, axis, velocity, acceleration):
        # v2 DOES honor vel/accel, but the firmware loads def_phil.h defaults at
        # startup. Encoding SET_MAX_VELOCITY_ACCELERATION (opcode 22) is deferred
        # until after basic-motion bring-up; defaults are safe meanwhile.
        return

    def set_axis_enable_disable(self, axis, status):
        # SET_AXIS_DISABLE_ENABLE: payload [axis, status]; status 1=disable,0=enable
        # (matches stock squid). Used by home_arms to home one motor at a time.
        self._send(SET_AXIS_DISABLE_ENABLE, axis, status)

    def close(self):
        self._stop = True
        try:
            self._reader.join(timeout=1)
        finally:
            self.serial.close()
