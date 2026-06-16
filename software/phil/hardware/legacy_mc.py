"""Legacy Microcontroller driver for the firmware flashed on this Phil Teensy.

The Teensy is running an OLDER octopi protocol than this repo's
``control/microcontroller.py`` expects. Empirically determined over USB:

  * command frame  = 6 bytes:  [cmd_id, opcode, p0, p1, p2, crc8]
                     (repo uses 8: [cmd_id, opcode, p0..p3, ?, crc8])
  * status packet  = 20 bytes: [cmd_id, status, X(4 BE), Y(4 BE), Z(4 BE),
                                 theta(4 BE), buttons, pad]  (no trailing CRC)
                     (repo uses 24, with 4 reserved bytes before the CRC)
  * CRC            = CRC-8/CCITT (poly 0x07), same as the repo / firmware crc8.cpp
  * opcodes        = same numbering as control._def.CMD_SET (MOVE_X=0 verified;
                     status COMPLETED=0 / IN_PROGRESS=1 / CHECKSUM_ERROR=2)

Because the command payload is only 3 bytes, positions are sent as a 24-bit
signed value (vs the repo's 32-bit).

This class implements the subset of the Microcontroller interface that
:class:`phil.robot.PhilRobot` uses, with a background read thread that
keeps byte-alignment on the delimiter-less 20-byte stream.
"""
from __future__ import annotations

import threading
import time

import serial
import serial.tools.list_ports as lp

CMD_LENGTH = 6
MSG_LENGTH = 20
POS_OFFSETS = {"X": 2, "Y": 6, "Z": 10, "THETA": 14}

# --- units -----------------------------------------------------------------
# Measured on this firmware: a relative MOVE command value V produces a
# firmware position change of V*256, i.e. the firmware microstepping is 256 and
# *commands are in full-steps* while *position is reported in microsteps*.
# The rest of the phil package (constants.py / PhilRobot) speaks "repo usteps"
# at MICROSTEPPING_DEFAULT = 8 (1600 usteps/mm for X,Y). To keep all that math
# unchanged, this driver converts at its boundary:
#     legacy command (full-steps) = repo_usteps / REPO_MICROSTEPPING
#     repo_usteps (reported)      = firmware_microsteps / (LEGACY/REPO)
LEGACY_MICROSTEPPING = 256
REPO_MICROSTEPPING = 8
CMD_DIVISOR = LEGACY_MICROSTEPPING // REPO_MICROSTEPPING   # 32: repo_ustep<->fw microstep
FULLSTEP_DIVISOR = REPO_MICROSTEPPING                      # 8 : repo_ustep -> full-step cmd

# opcodes (match control._def.CMD_SET)
MOVE_X, MOVE_Y, MOVE_Z = 0, 1, 2
HOME_OR_ZERO = 5
MOVETO_X, MOVETO_Y, MOVETO_Z = 6, 7, 8
SET_MAX_VELOCITY_ACCELERATION = 22
INITIALIZE = 254
RESET = 255

# HOME_OR_ZERO payload conventions (repo def): nominal home vs set-zero
HOME_NEGATIVE, HOME_POSITIVE, HOME_OR_ZERO_ZERO = 1, 2, 3
AXIS = {"X": 0, "Y": 1, "Z": 2}

_CRC_TABLE = [
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3,
]


def crc8(data) -> int:
    v = 0
    for b in data:
        v = _CRC_TABLE[v ^ b]
    return v


def _s24(value: int) -> bytes:
    """Signed 24-bit big-endian (clamped to the +/-8388607 range)."""
    value = max(-8388608, min(8388607, int(value)))
    return (value & 0xFFFFFF).to_bytes(3, "big")


def find_controller(version="Teensy", sn=None):
    # STRICT when an SN is requested: match ONLY that serial number, and return None if
    # it is absent (the caller raises -> fail loud). Falling back to "any Teensyduino"
    # when an SN was given let Phil seize the MICROSCOPE's Teensy whenever it enumerated
    # first, and Phil's jogs then drove the microscope stage (SAFETY, 2026-06-15). Two
    # boards on the bus are indistinguishable except by SN.
    ports = list(lp.comports())
    if sn:
        for p in ports:
            if p.serial_number == sn:
                return p.device
        return None
    # No SN given: auto-detect a SINGLE Teensy, but REFUSE to guess when several are
    # present -- the microscope stage is a second Teensy, and guessing once drove it.
    teensies = [p for p in ports
                if p.manufacturer == "Teensyduino" or p.description == "Arduino Due"]
    if len(teensies) > 1:
        raise IOError(
            "multiple Teensy controllers found and no serial number given — refusing to "
            "guess (one of these is the microscope stage). Set PHIL_SN=<serial> or pass "
            f"controller_sn. Found SNs: {[p.serial_number for p in teensies]}")
    return teensies[0].device if teensies else None


class LegacyMicrocontroller:
    def __init__(self, version="Teensy", sn=None, port=None):
        self.port = port or find_controller(version, sn)
        if self.port is None:
            raise IOError(f"Phil Teensy not found on USB (requested sn={sn!r}) — "
                          f"refusing to auto-grab another board.")
        self.serial = serial.Serial(self.port, 2000000, timeout=1)
        time.sleep(0.2)

        self._cmd_id = 0
        self._cmd_id_mcu = None
        self._cmd_execution_status = None
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
        """Find a 20-byte boundary using the echoed cmd_id, then parse latest."""
        buf = self._buf
        if len(buf) < MSG_LENGTH * 2:
            return
        target = self._cmd_id  # the controller echoes the last command id
        best = None
        # prefer alignment where byte0 == our last cmd_id at stride MSG_LENGTH
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
            # keep only the tail to bound memory
            if len(buf) > MSG_LENGTH * 8:
                del buf[:-MSG_LENGTH * 4]
            return
        # parse the most recent complete packet at this alignment
        last = best + ((len(buf) - best) // MSG_LENGTH - 1) * MSG_LENGTH
        pk = bytes(buf[last:last + MSG_LENGTH])
        self._cmd_id_mcu = pk[0]
        self._cmd_execution_status = pk[1]
        self.x_pos = int.from_bytes(pk[2:6], "big", signed=True)
        self.y_pos = int.from_bytes(pk[6:10], "big", signed=True)
        self.z_pos = int.from_bytes(pk[10:14], "big", signed=True)
        self.theta_pos = int.from_bytes(pk[14:18], "big", signed=True)
        self.button_and_switch_state = pk[18]
        if self.mcu_cmd_execution_in_progress and pk[0] == self._cmd_id and pk[1] == 0:
            self.mcu_cmd_execution_in_progress = False
        # drop consumed bytes, keep a small tail
        del buf[:last]

    # -------------------------------------------------------------- sending
    def _send(self, opcode, p0=0, p1=0, p2=0, expect_ack=True):
        self._cmd_id = (self._cmd_id + 1) % 256
        body = bytes([self._cmd_id, opcode, p0 & 0xFF, p1 & 0xFF, p2 & 0xFF])
        frame = body + bytes([crc8(body)])
        if expect_ack:
            self.mcu_cmd_execution_in_progress = True
        self.serial.write(frame)

    def _send_pos(self, opcode, repo_usteps):
        # repo usteps (microstepping 8) -> legacy full-step command
        cmd = int(round(repo_usteps / FULLSTEP_DIVISOR))
        b = _s24(cmd)
        self._send(opcode, b[0], b[1], b[2])

    # ----------------------------------------------------- lifecycle/config
    def reset(self):
        self._send(RESET, expect_ack=False)

    def initialize_drivers(self):
        self._send(INITIALIZE, expect_ack=False)

    def configure_actuators(self):
        # The legacy firmware applies its own defaults at INITIALIZE; the
        # repo-style multi-field configure does not fit a 3-byte payload, so
        # we leave the firmware defaults in place (they produce smooth motion).
        pass

    def set_max_velocity_acceleration(self, axis, velocity, acceleration):
        # Not safely encodable in 3 payload bytes on this firmware; skip so we
        # don't accidentally zero the motion profile. Firmware default is used.
        return

    def set_axis_enable_disable(self, axis, status):
        # opcode for enable/disable is not validated on this firmware; no-op.
        return

    # ------------------------------------------------------------- motion
    def move_x_usteps(self, u): self._send_pos(MOVE_X, u)
    def move_y_usteps(self, u): self._send_pos(MOVE_Y, u)
    def move_z_usteps(self, u): self._send_pos(MOVE_Z, u)
    def move_x_to_usteps(self, u): self._send_pos(MOVETO_X, u)
    def move_y_to_usteps(self, u): self._send_pos(MOVETO_Y, u)
    def move_z_to_usteps(self, u): self._send_pos(MOVETO_Z, u)

    def home_x(self): self._send(HOME_OR_ZERO, AXIS["X"], HOME_NEGATIVE)
    def home_y(self): self._send(HOME_OR_ZERO, AXIS["Y"], HOME_NEGATIVE)
    def home_z(self): self._send(HOME_OR_ZERO, AXIS["Z"], HOME_NEGATIVE)
    def home_xy(self):
        self.home_x()
        self.wait_till_operation_is_completed()
        self.home_y()
    def zero_x(self): self._send(HOME_OR_ZERO, AXIS["X"], HOME_OR_ZERO_ZERO)
    def zero_y(self): self._send(HOME_OR_ZERO, AXIS["Y"], HOME_OR_ZERO_ZERO)
    def zero_z(self): self._send(HOME_OR_ZERO, AXIS["Z"], HOME_OR_ZERO_ZERO)

    # ------------------------------------------------------------- status
    def get_pos(self):
        # firmware microsteps -> repo usteps (microstepping 8)
        return (int(round(self.x_pos / CMD_DIVISOR)),
                int(round(self.y_pos / CMD_DIVISOR)),
                int(round(self.z_pos / CMD_DIVISOR)),
                self.theta_pos)

    def is_busy(self):
        return self.mcu_cmd_execution_in_progress

    def wait_till_operation_is_completed(self, timeout=5):
        t0 = time.time()
        while self.is_busy() and time.time() - t0 < timeout:
            time.sleep(0.01)
        # legacy firmware does not always re-echo COMPLETED reliably; clear the
        # flag on timeout so callers are not blocked forever.
        self.mcu_cmd_execution_in_progress = False

    def close(self):
        self._stop = True
        try:
            self._reader.join(timeout=1)
        finally:
            self.serial.close()
