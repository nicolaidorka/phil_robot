"""PhilRobot - high-level, calibrated, smooth motion control for the Phil arm.

Phil is a 3-axis robot (X = left arm, Y = right arm, Z = up/down) driven
over USB by the Squid ``control.microcontroller.Microcontroller``.  This
module wraps that low-level driver with:

  * a clean connect / home / configure sequence,
  * mm<->microstep conversion and absolute/relative XYZ moves,
  * a smooth motion profile (velocity + acceleration limits),
  * soft travel limits to avoid crashes,
  * ``goto_well("A1")`` using the labware + calibration, with a safe
    lift-travel-descend sequence,
  * a no-hardware simulation backend for testing the geometry/CLI.

Z convention: +Z is UP (lifted, safe); smaller Z is DOWN (into the well).

Typical use (run from the ``software/`` directory so the Squid
configuration*.ini is found, same as test_20240823.py)::

    from phil import PhilRobot
    bot = PhilRobot()           # loads default labware + saved calibration
    bot.connect()
    bot.home()
    bot.goto_well("D6")
    bot.move_relative(dz=-1.0)  # nudge down 1 mm
    print(bot.position())
    bot.close()
"""
from __future__ import annotations

import json
import os
import time

from . import constants as C
from . import paths
from .geometry.calibration import Calibration
from .geometry.teach import TeachTable, DEFAULT_TEACH_PATH
from .geometry.well_plate import WellPlate

try:
    from .geometry.well_map import WellMap          # needs scipy; optional
except Exception:                          # pragma: no cover
    WellMap = None
try:
    from .geometry.kinematics import KinematicModel  # needs scipy; optional
except Exception:                          # pragma: no cover
    KinematicModel = None


# --- joint-frame correction (affine over the raw model prediction) ---------
# Xcmd = a*Xm + b*Ym + cx ;  Ycmd = d*Xm + e*Ym + cy .  Recovers a counter
# shift / small plate move-or-rotation after a power-cycle or bump WITHOUT
# touching the 5-bar geometry. Identity = no correction (exact pass-through).
IDENTITY_CORRECTION = {"a": 1.0, "b": 0.0, "cx": 0.0, "d": 0.0, "e": 1.0, "cy": 0.0}
# Guardrails so a noisy/degenerate fit can never blow up a goto.
_MAX_LINEAR_DEV = 0.02      # |a-1|,|b|,|d|,|e-1| beyond this -> reject scale/shear
_MAX_SPAN_USTEPS = 12       # implied correction range over the plate beyond this -> reject
_SNAP_IDENTITY = 0.5        # a fit this close to identity is snapped to exact identity


def _is_identity(fc):
    return (fc["a"] == 1.0 and fc["b"] == 0.0 and fc["cx"] == 0.0
            and fc["d"] == 0.0 and fc["e"] == 1.0 and fc["cy"] == 0.0)


def _is_translation_only(fc):
    """True if the correction is a pure (cx,cy) shift with an identity linear part.

    reanchor() / a 1-2 point fit produce this (power-cycle / bump recovery): a
    rigid shift of the whole joint frame. ``anchor fit`` (>=3 points) produces a
    full affine whose (cx,cy) intercept is NOT a standalone translation.
    """
    return fc["a"] == 1.0 and fc["b"] == 0.0 and fc["d"] == 0.0 and fc["e"] == 1.0


def _apply_correction(fc, xm, ym):
    return (int(round(fc["a"] * xm + fc["b"] * ym + fc["cx"])),
            int(round(fc["d"] * xm + fc["e"] * ym + fc["cy"])))


# ---------------------------------------------------------------------------
# Simulation backend (mimics the bits of Microcontroller that PhilRobot uses)
# ---------------------------------------------------------------------------
class SimulatedBackend:
    """In-memory stand-in for the Squid Microcontroller (no hardware)."""

    def __init__(self):
        self.x_pos = self.y_pos = self.z_pos = self.theta_pos = 0
        self._busy = False

    # connection lifecycle ---------------------------------------------------
    def reset(self): pass
    def initialize_drivers(self): pass
    def configure_actuators(self): pass
    def close(self): pass
    def set_callback(self, fn): pass

    # status -----------------------------------------------------------------
    def is_busy(self): return False
    def wait_till_operation_is_completed(self, timeout=5): pass
    def get_pos(self): return self.x_pos, self.y_pos, self.z_pos, self.theta_pos

    # config -----------------------------------------------------------------
    def set_max_velocity_acceleration(self, axis, vel, acc): pass
    def set_axis_enable_disable(self, axis, status): pass

    # motion -----------------------------------------------------------------
    def move_x_to_usteps(self, u): self.x_pos = u
    def move_y_to_usteps(self, u): self.y_pos = u
    def move_z_to_usteps(self, u): self.z_pos = u
    # Relative jogs raise the reported count for a + value, matching the legacy
    # firmware/teach convention (Up arrow = +dx records an increasing X). This
    # MUST agree with the absolute moves above so the interleaved approach
    # (absolute pre-position + relative creep) converges on the sim too.
    def move_x_usteps(self, u): self.x_pos += u
    def move_y_usteps(self, u): self.y_pos += u
    def move_z_usteps(self, u): self.z_pos += u
    def home_x(self): self.x_pos = 0
    def home_y(self): self.y_pos = 0
    def home_z(self): self.z_pos = 0
    def home_xy(self): self.x_pos = self.y_pos = 0
    def zero_x(self): self.x_pos = 0
    def zero_y(self): self.y_pos = 0
    def zero_z(self): self.z_pos = 0


def _connect_real_backend(version="Arduino Due", sn=None):
    """Lazily import and connect the real Squid Microcontroller over USB."""
    # Imported lazily: pulls in Qt and requires a configuration*.ini in cwd.
    from control.microcontroller import Microcontroller
    return Microcontroller(version=version, sn=sn)


class PhilHandshakeError(RuntimeError):
    """Raised when the controller streams feedback but never ACKs commands."""


# ---------------------------------------------------------------------------
class PhilRobot:
    def __init__(self, labware_path=None, calibration_path=None,
                 plate=None, calibration=None, simulate=False,
                 controller_version="Teensy", controller_sn=None,
                 backend="legacy", teach_path=None):
        self.plate = plate or WellPlate.load(labware_path)
        if calibration is not None:
            self.calibration = calibration
        else:
            cal = Calibration.load(calibration_path, plate=self.plate)
            # no saved reference points -> start from the nominal model
            self.calibration = cal if cal.is_fitted else Calibration.nominal(self.plate)
        self.calibration_path = calibration_path

        # teach-and-replay table (the right model for the articulated 5-bar arm)
        self.teach_path = teach_path
        self.teach_table = TeachTable.load(teach_path)
        # nonlinear curve-fit map (RBF) from taught wells -> joints
        self.well_map = WellMap(self.plate, self.teach_table) if WellMap else None
        if self.well_map:
            self.well_map.fit()
        # 5-bar kinematic model (best: real geometry -> any well/any labware)
        self.kin_model = KinematicModel.load() if KinematicModel else None
        # joint-frame correction to recover the calibration after a power-cycle
        # or bump (set via reanchor()/anchor, persisted to disk). Affine over the
        # raw model prediction; identity until fitted.
        cfg_dir = os.path.dirname(teach_path or paths.DEFAULT_TEACH_PATH)
        self._frame_path = os.path.join(cfg_dir, paths.FRAME_FILENAME)
        self.frame_correction = dict(IDENTITY_CORRECTION)
        self._anchor_pts = {}           # well -> (measured_xy, model_xy) for fit
        self._last_joints = None        # last commanded (X,Y) for power-cycle detection
        self.frame_suspect = False      # True if the counter looks reset on connect
        self._load_frame()

        self.simulate = simulate
        # backend: 'legacy' (this Phil's older 6-byte/20-byte firmware),
        # 'stock' (repo control.microcontroller), or forced 'sim' when simulate.
        self.backend = "sim" if simulate else backend
        self._controller_version = controller_version
        self._controller_sn = controller_sn
        self.mc = None
        self.connected = False
        self.homed = False

        # Joint-count scale. The v2 microstep firmware reports/commands raw
        # 256-microsteps, so its counts are 32x the legacy 8-ustep scale
        # (legacy ~5.5 counts/mm at the tip -> v2 ~175 counts/mm). Rescale every
        # count-based motion tunable by this factor so legacy and v2 produce the
        # SAME physical legs / run-up / tolerances. (1 for legacy & sim.)
        self._ustep_scale = 32 if self.backend == "v2" else 1
        if self._ustep_scale != 1:
            cls, s = type(self), self._ustep_scale
            self.MOVE_CHUNK_USTEPS = cls.MOVE_CHUNK_USTEPS * s
            self.APPROACH_PRE_USTEPS = cls.APPROACH_PRE_USTEPS * s
            self.APPROACH_CONFIRM_TOL = cls.APPROACH_CONFIRM_TOL * s
            self.APPROACH_OK_USTEPS = cls.APPROACH_OK_USTEPS * s
            self.APPROACH_MAX_CORRECTION = cls.APPROACH_MAX_CORRECTION * s
            self.FRAME_RESET_THRESHOLD = cls.FRAME_RESET_THRESHOLD * s
            # v2 honors accel ramps at the (low) bring-up velocity, so a chunk leg
            # takes longer than the legacy fixed-profile 2.0 s settle budget.
            self.APPROACH_CONFIRM_TIMEOUT = 8.0

        if self.backend == "v2":
            # A persisted phil_frame.json reanchor offset is in LEGACY ustep units;
            # applied on v2 it would be a 32x-too-small shift. Start from identity and
            # re-anchor fresh on v2 (the geometry is re-taught at the finer scale anyway).
            self.frame_correction = dict(IDENTITY_CORRECTION)
            self._last_joints = None
            self.frame_suspect = False

        # commanded position in robot mm (authoritative once homed/zeroed)
        self._pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}

        self.max_velocity = dict(C.DEFAULT_MAX_VELOCITY)
        self.max_acceleration = dict(C.DEFAULT_MAX_ACCELERATION)
        self.soft_limits = {k: tuple(v) for k, v in C.SOFT_LIMITS_MM.items()}
        self.move_timeout_s = 20.0

    # ----------------------------------------------------------- connection
    def connect(self):
        if self.connected:
            return
        if self.backend == "sim":
            self.mc = SimulatedBackend()
        elif self.backend == "legacy":
            from .hardware.legacy_mc import LegacyMicrocontroller
            self.mc = LegacyMicrocontroller(
                version=self._controller_version, sn=self._controller_sn)
        elif self.backend == "v2":
            from .hardware.v2_mc import V2Microcontroller
            self.mc = V2Microcontroller(
                version=self._controller_version, sn=self._controller_sn)
        else:
            self.mc = _connect_real_backend(self._controller_version, self._controller_sn)

        if self.backend == "legacy":
            # Preserve the firmware's persisted joint frame across reconnects:
            # the Teensy keeps its position + driver config from power-on, and
            # reset()/initialize_drivers() would zero it. Use set_home() to zero
            # deliberately. Motion works without host init (verified).
            time.sleep(0.3)
            self.connected = True
            print(f"PhilRobot connected (backend=legacy, frame preserved). "
                  f"{self.teach_table.summary()}")
            self._check_frame()
            return

        if self.backend == "v2":
            # v2 microstep firmware: apply def_phil.h driver config (motor current,
            # 256 microstepping, velocity/accel ramps) via INITIALIZE so the TMC2660
            # drives at the intended (low) current. Do NOT reset() -- avoid zeroing
            # the persisted joint frame; deliberate zeroing is via set_home(). The
            # command grid is now microstep-fine (the whole point of the reflash).
            time.sleep(0.3)
            self.mc.initialize_drivers()
            time.sleep(0.5)
            self.mc.configure_actuators()
            self.connected = True
            print(f"PhilRobot connected (backend=v2, microstep). "
                  f"{self.teach_table.summary()}")
            self._check_frame()
            return

        self.mc.reset()
        time.sleep(0.5)
        self.mc.initialize_drivers()
        time.sleep(0.5)

        if self.backend == "stock":
            # The stock driver's wait calls sys.exit(1) on a command-ack timeout;
            # verify the handshake first so a mismatch raises cleanly instead.
            if not self._command_handshake_ok():
                self.close()
                raise PhilHandshakeError(
                    "Connected and receiving position feedback, but the controller "
                    "is not acknowledging commands. The stock 8-byte/24-byte protocol "
                    "does not match this firmware. Use backend='legacy' (this Phil's "
                    "6-byte/20-byte firmware) instead, or reflash to match the repo.")

        self.mc.configure_actuators()
        time.sleep(0.3)
        self.connected = True
        if self.backend == "stock":
            self.apply_motion_profile()
        print(f"PhilRobot connected (backend={self.backend}). "
              f"{self.teach_table.summary()}")

    def _command_handshake_ok(self, timeout_s: float = 3.0) -> bool:
        """Send one no-motion command and check the controller ACKs it."""
        # SET_MAX_VELOCITY_ACCELERATION moves nothing; safe to probe with.
        self.mc.set_max_velocity_acceleration(
            C.AXIS_X, self.max_velocity["X"], self.max_acceleration["X"])
        t0 = time.time()
        while self.mc.is_busy() and time.time() - t0 < timeout_s:
            time.sleep(0.02)
        return not self.mc.is_busy()

    def close(self):
        if self.mc is not None:
            try:
                if self.connected and not self.simulate:
                    j = self.joint_position()      # checkpoint last pose for frame detection
                    self._last_joints = (j["X"], j["Y"])
                    self._save_frame()
            except Exception:
                pass
            try:
                self.mc.close()
            finally:
                self.mc = None
                self.connected = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    def _require(self):
        if not self.connected:
            raise RuntimeError("not connected - call connect() first")

    # ------------------------------------------------------- motion profile
    def apply_motion_profile(self):
        """Push velocity/acceleration limits to the firmware for smooth moves."""
        self._require()
        for axis_name, axis_id in (("X", C.AXIS_X), ("Y", C.AXIS_Y), ("Z", C.AXIS_Z)):
            self.mc.set_max_velocity_acceleration(
                axis_id, self.max_velocity[axis_name], self.max_acceleration[axis_name])
            self._wait()

    def set_speed(self, factor: float):
        """Scale velocity for all axes (1.0 = defaults). Acceleration unchanged."""
        factor = max(0.05, min(1.0, float(factor)))
        for k in self.max_velocity:
            self.max_velocity[k] = C.DEFAULT_MAX_VELOCITY[k] * factor
        if self.connected:
            self.apply_motion_profile()

    # ----------------------------------------------------------------- wait
    def _wait(self):
        self.mc.wait_till_operation_is_completed(self.move_timeout_s)

    def _confirm_joints(self, x=None, y=None, tol=None, timeout=None):
        """Poll the live joints until the commanded axes actually reach target.

        The legacy firmware only reports completion for the LAST command sent, so
        after a two-axis move ``_wait`` can return while the longer-travel axis is
        still moving (`legacy_mc.py` clears the busy flag on the latest cmd_id).
        This re-reads the reported counts and waits until both are within ``tol``.
        Returns True on arrival, False on timeout.
        """
        tol = self.APPROACH_CONFIRM_TOL if tol is None else tol
        timeout = self.APPROACH_CONFIRM_TIMEOUT if timeout is None else timeout
        t0 = time.time()
        hits = 0
        while True:
            j = self.joint_position()
            okx = x is None or abs(j["X"] - int(x)) <= tol
            oky = y is None or abs(j["Y"] - int(y)) <= tol
            if okx and oky:
                hits += 1
                if hits >= 3:        # require the reading to HOLD: a single
                    return True      # in-tol read can be stale/mid-move (the
            else:                    # firmware reports the last cmd, not motion)
                hits = 0
            if time.time() - t0 >= timeout:
                return hits > 0      # close enough if we ever saw target, else timeout
            time.sleep(0.03)

    # --------------------------------------------------------------- homing
    def home(self, z_lift_mm: float = 1.0):
        """Full homing: Z first (then lift), then both arms, then zero.

        Mirrors the proven sequence in test_20240823.py: home one arm at a
        time with the other motor disabled to avoid interference.
        """
        self._require()
        self.home_z(z_lift_mm)
        self.home_arms()
        self.homed = True
        print(f"homed. position={self.position()}")

    def home_z(self, z_lift_mm: float = 1.0):
        self._require()
        self.mc.home_z()
        self._wait()
        self.mc.zero_z()
        self._wait()
        self._pos["Z"] = 0.0
        if z_lift_mm:
            self._move_axis_abs("Z", z_lift_mm)

    def home_arms(self):
        """Home Y then X, one motor enabled at a time, then zero both."""
        self._require()
        # home Y (right arm) with X disabled. Firmware polarity: 0=disable, 1=enable
        # (the legacy driver no-ops this; v2 honors it, so use the firmware convention).
        self.mc.set_axis_enable_disable(C.AXIS_X, 0)
        self.mc.set_axis_enable_disable(C.AXIS_Y, 1)
        self.mc.home_y()
        self._wait()
        self.mc.zero_y()
        self._wait()
        # home X (left arm) with Y disabled
        self.mc.set_axis_enable_disable(C.AXIS_X, 1)
        self.mc.set_axis_enable_disable(C.AXIS_Y, 0)
        self.mc.home_x()
        self._wait()
        self.mc.zero_x()
        self._wait()
        # re-enable both arms
        self.mc.set_axis_enable_disable(C.AXIS_X, 1)
        self.mc.set_axis_enable_disable(C.AXIS_Y, 1)
        self._pos["X"] = 0.0
        self._pos["Y"] = 0.0

    # ------------------------------------------------------------- position
    def position(self) -> dict:
        """Commanded robot position in mm (authoritative after homing)."""
        return dict(self._pos)

    def read_position(self) -> dict:
        """Position read back from the firmware (mm), for verification."""
        self._require()
        x, y, z, _ = self.mc.get_pos()
        # mm<->ustep math in constants assumes microstepping 8 (legacy repo-usteps);
        # the v2 firmware reports raw 256-microsteps, so divide back by the scale.
        s = self._ustep_scale
        return {"X": C.usteps_to_mm("X", x / s),
                "Y": C.usteps_to_mm("Y", y / s),
                "Z": C.usteps_to_mm("Z", z / s)}

    # ------------------------------------------------------------- limits
    def _check_limit(self, axis: str, mm: float):
        lo, hi = self.soft_limits[axis]
        if not (lo <= mm <= hi):
            raise ValueError(
                f"{axis} target {mm:.3f} mm outside soft limit [{lo}, {hi}] mm. "
                f"Adjust calibration or PhilRobot.soft_limits if this is intended.")

    # ------------------------------------------------------- low-level move
    def _move_axis_abs(self, axis: str, mm: float):
        self._check_limit(axis, mm)
        # constants give legacy repo-usteps (microstepping 8); v2 commands raw
        # 256-microsteps, so scale up to the firmware's unit.
        usteps = C.mm_to_usteps(axis, mm) * self._ustep_scale
        {"X": self.mc.move_x_to_usteps,
         "Y": self.mc.move_y_to_usteps,
         "Z": self.mc.move_z_to_usteps}[axis](usteps)
        self._wait()
        self._pos[axis] = mm

    # --------------------------------------------------------- public moves
    def move_to(self, x=None, y=None, z=None, safe=True):
        """Absolute move. With ``safe`` and an XY change, lift Z, travel, descend."""
        self._require()
        tx = self._pos["X"] if x is None else float(x)
        ty = self._pos["Y"] if y is None else float(y)
        tz = self._pos["Z"] if z is None else float(z)
        for a, v in (("X", tx), ("Y", ty), ("Z", tz)):
            self._check_limit(a, v)

        xy_changes = (x is not None and tx != self._pos["X"]) or \
                     (y is not None and ty != self._pos["Y"])

        if safe and xy_changes:
            travel_z = max(self.calibration.z_safe_mm, tz, self._pos["Z"])
            self._check_limit("Z", travel_z)
            self._move_axis_abs("Z", travel_z)   # 1. lift
            self._move_axis_abs("X", tx)          # 2. travel
            self._move_axis_abs("Y", ty)
            self._move_axis_abs("Z", tz)          # 3. descend
        else:
            if z is not None and tz > self._pos["Z"]:
                self._move_axis_abs("Z", tz)      # lift before XY if asked
            if x is not None:
                self._move_axis_abs("X", tx)
            if y is not None:
                self._move_axis_abs("Y", ty)
            if z is not None and tz <= self._pos["Z"]:
                self._move_axis_abs("Z", tz)
        return self.position()

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, safe=False):
        return self.move_to(self._pos["X"] + dx, self._pos["Y"] + dy,
                            self._pos["Z"] + dz, safe=safe)

    # convenience single-axis jogs
    def move_x(self, dx, safe=False): return self.move_relative(dx=dx, safe=safe)
    def move_y(self, dy, safe=False): return self.move_relative(dy=dy, safe=safe)
    def move_z(self, dz, safe=False): return self.move_relative(dz=dz, safe=safe)

    # ------------------------------------------------- joint space (5-bar arm)
    def joint_position(self) -> dict:
        """Actual joint positions read from the firmware, in repo usteps."""
        self._require()
        x, y, z, _ = self.mc.get_pos()
        return {"X": int(x), "Y": int(y), "Z": int(z)}

    def jog_joint(self, dx=0, dy=0, dz=0):
        """Relative joint jog in usteps (used to position the arm for teaching)."""
        self._require()
        if dx:
            self.mc.move_x_usteps(int(dx)); self._wait()
        if dy:
            self.mc.move_y_usteps(int(dy)); self._wait()
        if dz:
            self.mc.move_z_usteps(int(dz)); self._wait()
        return self.joint_position()

    # The legacy firmware ignores velocity/accel limits and accelerates to a fixed
    # top speed on long moves -- exactly where the steppers lack torque and skip
    # steps (silent drift, since there are no encoders). We can't slow it directly,
    # but a SHORT move never reaches that top speed. So break every XY traverse into
    # legs of at most this many usteps: the arm stays in the low-speed accel/decel
    # band the whole way. Smaller = safer but slower.
    MOVE_CHUNK_USTEPS = 40

    def _move_joints_to(self, x=None, y=None, z=None, coordinated=True):
        """Absolute joint move in usteps, broken into short coordinated legs so the
        fixed-profile firmware never reaches the high speed where it loses steps."""
        self._require()
        if x is not None or y is not None:
            cur = self.joint_position()
            tx = int(x) if x is not None else cur["X"]
            ty = int(y) if y is not None else cur["Y"]
            x0, y0 = cur["X"], cur["Y"]
            dist = max(abs(tx - x0), abs(ty - y0))
            legs = max(1, -(-dist // self.MOVE_CHUNK_USTEPS))   # ceil division
            for i in range(1, legs + 1):
                ix = int(round(x0 + (tx - x0) * i / legs))
                iy = int(round(y0 + (ty - y0) * i / legs))
                self.mc.move_x_to_usteps(ix)
                self.mc.move_y_to_usteps(iy)
                self._wait()
                # Confirm EACH leg lands before sending the next. The legacy driver
                # only ACKs the last command, so without this the legs pile up into
                # one fast move (defeating the speed limit) or get cut short -- the
                # accepted-slop-seeds-the-next-move drift that grows over a session.
                self._confirm_joints(x=ix, y=iy)
        if z is not None:
            self.mc.move_z_to_usteps(int(z)); self._wait()

    # Final-approach tuning. Teaching reaches a well with small RELATIVE +X/+Y jogs
    # (Up/Right arrows); goto must finish the SAME way so the open-loop joints
    # settle in the same backlash state for the same count. The approach advances
    # the two arms TOGETHER (alternating small steps) so the outlet closes in along
    # a coordinated diagonal instead of an L-shaped single-joint swing that strains
    # the 5-bar linkage. (The legacy firmware has a fixed motion profile and ignores
    # velocity/accel, so coordination is by interleaved path, not by speed.)
    # Run-up distance for the final approach. The arm must arrive with MOMENTUM so
    # it lands in the repeatable kinetic-friction regime; a short creep stops inside
    # the static-friction (stiction) band and settles at a random point -> the
    # different-direction-each-time error on close well-to-well moves. So the final
    # leg is ONE continuous move this long (NOT chunked), big enough to build
    # momentum but short enough to stay under the step-loss speed. Approaching from
    # -X,-Y also fixes the backlash direction (must exceed the backlash gap too).
    APPROACH_PRE_USTEPS = 80
    # The firmware commands in whole full-steps (8 repo-usteps) and reports the
    # position quantized to that grid, so a commanded count can read back up to ~4
    # usteps off. Confirm within one full-step, and never block long: this is a
    # best-effort settle check on top of _wait(), not a hard gate.
    APPROACH_CONFIRM_TOL = 8      # usteps; both axes within this of target = arrived
    APPROACH_CONFIRM_TIMEOUT = 2.0  # seconds; give up and proceed (don't freeze)
    APPROACH_OK_USTEPS = 8        # accept within one full-step: the firmware commands on an
                                  # 8-ustep grid, so it CANNOT deliberately stop tighter
    APPROACH_MAX_ITERS = 6        # closed-loop correction passes (noisy poses need more)
    APPROACH_MAX_CORRECTION = 30  # guard: a bigger "error" is a bad read, don't chase it
    APPROACH_DAMPING = 0.5        # near target, apply only this fraction (filter read noise)
    APPROACH_READS = 5            # median this many reads -> reject mid-move/stale samples

    def _read_joints_settled(self, n=None, dt=0.02):
        """Median of several position reads. The reader thread hands back whatever
        packet arrived last (sometimes mid-move or stale), so one read is noisy; the
        median of a short burst is the stable position."""
        n = self.APPROACH_READS if n is None else n
        xs, ys = [], []
        for _ in range(n):
            j = self.joint_position(); xs.append(j["X"]); ys.append(j["Y"]); time.sleep(dt)
        xs.sort(); ys.sort()
        return xs[n // 2], ys[n // 2]

    def _approach_joints(self, x, y, approach=(1, 1)):
        """Approach (x,y) from a fixed direction, then CLOSED-LOOP correct on the
        position readout.

        ``approach`` is the per-axis sign: +1 = come from the -side and close in +;
        -1 = come from the +side and close in -. Default (+1,+1) is the canonical
        -X,-Y -> +X,+Y close-in. A per-well override matches how a well was taught,
        closing the count-vs-physical backlash gap that leaves some wells (e.g. E12)
        a fixed amount off even when the count reads on-target.

        The firmware overshoots the commanded count by a small repeatable amount, so
        each pass reads the MEDIAN residual vs the true target and re-commands to
        cancel it. Hard-won details: the accept band is one full-step (8) because the
        firmware can't command finer; a big error gets the full correction while near
        target it's damped (so it neither chases noise nor rounds to a no-op); and a
        >MAX_CORRECTION 'error' is a bad read and is ignored so it can't fling the arm."""
        x, y = int(x), int(y)
        sx, sy = approach
        pre = self.APPROACH_PRE_USTEPS
        cx, cy = x, y
        for _ in range(self.APPROACH_MAX_ITERS):
            self._move_joints_to(x=cx - sx * pre, y=cy - sy * pre)  # pre-position on matching side
            self.mc.move_x_to_usteps(cx)                            # close in along taught direction
            self.mc.move_y_to_usteps(cy)
            self._wait()
            self._confirm_joints(x=cx, y=cy)
            mx, my = self._read_joints_settled()                    # median read (reject noise)
            ex, ey = x - mx, y - my                                 # residual vs the TRUE target
            if abs(ex) <= self.APPROACH_OK_USTEPS and abs(ey) <= self.APPROACH_OK_USTEPS:
                break
            if abs(ex) > self.APPROACH_MAX_CORRECTION or abs(ey) > self.APPROACH_MAX_CORRECTION:
                break                                               # implausible read -> trust move
            big = 2 * self.APPROACH_OK_USTEPS    # scales with backend (legacy 16, v2 512)
            a = 1.0 if (abs(ex) > big or abs(ey) > big) else self.APPROACH_DAMPING
            cx += int(round(a * ex)); cy += int(round(a * ey))

    # ------------------------------------------------------------- home/zero
    def set_home(self):
        """Zero the joints at the CURRENT pose (manual home reference).

        Safe: uses reset (which zeros the position counters with no motion),
        then re-initializes the drivers. Jog the arm to a repeatable physical
        reference first, then call this so taught wells survive across sessions.
        """
        self._require()
        self.mc.reset()
        time.sleep(1.0)
        self.mc.initialize_drivers()
        time.sleep(1.0)
        self._pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.homed = True
        print(f"home set at current pose; joints now {self.joint_position()}")

    # --------------------------------------------------------------- teaching
    def teach_well(self, well_id: str):
        """Save current joints as this well, and feed the metric (affine) fit.

        Each taught well becomes both an exact replay point AND a reference for
        the plate-mm <-> joint affine map, so a few wells derive a metric system
        that also adapts to other labware via their JSON.
        """
        p = self.joint_position()
        self.teach_table.teach(well_id, p["X"], p["Y"], p["Z"])
        # also record for the mm<->joint affine (the "metric system")
        self.calibration.add_reference(
            well_id, (p["X"], p["Y"], p["Z"]), plate=self.plate)
        if self.well_map:
            self.well_map.fit()
        print(f"taught {well_id.upper()} @ joints X={p['X']} Y={p['Y']} Z={p['Z']}")
        if self.calibration.is_fitted and len(self.calibration.reference_points) >= 2:
            print("  metric map: " + self.calibration.summary())
        return p

    def predict_well(self, well_id: str, plate=None) -> dict:
        """Predicted joints for a well from the fitted map (RBF if available).

        ``plate`` lets you predict for a *different* labware (pass a WellPlate
        loaded from another JSON) using the same fitted map.
        """
        if self.kin_model and self.kin_model.is_fitted:
            try:
                return self.kin_model.predict(well_id, self.plate, target_plate=plate)
            except Exception:
                pass
        if self.well_map and self.well_map.is_fitted:
            return self.well_map.predict(well_id, plate=plate)
        x, y, z = self.calibration.well_to_robot(plate or self.plate, well_id)
        return {"X": int(round(x)), "Y": int(round(y)), "Z": int(round(z))}

    def set_travel_z(self, z_usteps=None):
        """Set the safe travel Z (usteps) for between-well moves; None=current."""
        if z_usteps is None:
            z_usteps = self.joint_position()["Z"]
        self.teach_table.z_travel_usteps = int(z_usteps)
        print(f"travel Z set to {self.teach_table.z_travel_usteps} usteps")

    # --------------------------------------------------------------- wells
    def well_position(self, well_id: str) -> dict:
        """Joint position (usteps) a well resolves to, without moving."""
        return self._resolve_well(well_id)[0]

    def _resolve_well(self, well_id: str):
        """Joint target: exact taught > 5-bar kinematics > RBF curve-fit > affine.

        The persisted joint-frame correction (from reanchor/anchor) has two roles
        that apply to different wells:

          * a pure translation (reanchor / power-cycle recovery) is a rigid shift
            of the whole joint frame -> applied to EVERY well, taught included, so
            the calibration survives a controller power-cycle.
          * a full ``anchor fit`` affine (scale/shear, edge refinement of the 5-bar
            model) is applied ONLY to model-derived (untaught) wells. A taught well
            is measured ground truth -- exact replay -- and must never be distorted
            by the model's edge-refinement affine.
        """
        tgt, src = self._resolve_raw(well_id)
        fc = self.frame_correction
        if _is_identity(fc):
            return tgt, src                      # exact pass-through (byte-identical)
        if src == "taught" and not _is_translation_only(fc):
            return tgt, src                      # don't apply the edge affine to truth
        x, y = _apply_correction(fc, tgt["X"], tgt["Y"])
        return {**tgt, "X": x, "Y": y}, src

    @property
    def joint_offset(self):
        """Back-compat: the translation part (cx, cy) of the frame correction."""
        return (self.frame_correction["cx"], self.frame_correction["cy"])

    def _resolve_raw(self, well_id: str):
        # A taught well is measured ground truth -> exact replay ALWAYS wins.
        # (The 5-bar model overfits the ~10 taught wells and mis-places untaught
        #  ones by up to ~4 mm at the edges, so a recorded well must beat it.)
        if self.teach_table.is_taught(well_id):
            return self.teach_table.joint_for_well(well_id, self.plate), "taught"
        # Not taught yet: fall back to the model so the arm still moves while you
        # teach. Once every well is taught, nothing reaches these branches.
        if self.kin_model and self.kin_model.is_fitted:
            try:
                return self.kin_model.predict(well_id, self.plate), "kinematics"
            except Exception:
                pass
        if self.well_map and self.well_map.is_fitted:
            return self.well_map.predict(well_id), "curve-fit"
        if self.calibration.is_fitted and len(self.calibration.reference_points) >= 3:
            return self.predict_well(well_id), "metric-affine"
        return self.teach_table.joint_for_well(well_id, self.plate), "interpolated"

    # ----------------------------------------------------- frame re-anchor
    FRAME_RESET_THRESHOLD = 80          # usteps; bigger jump on connect => reset

    def _load_frame(self):
        try:
            with open(self._frame_path) as f:
                d = json.load(f)
            if "a" in d:                          # new affine schema
                self.frame_correction = {k: float(d.get(k, IDENTITY_CORRECTION[k]))
                                         for k in IDENTITY_CORRECTION}
            else:                                 # back-compat: old {dx,dy} -> translation
                fc = dict(IDENTITY_CORRECTION)
                fc["cx"] = float(d.get("dx", 0)); fc["cy"] = float(d.get("dy", 0))
                self.frame_correction = fc
            if "last_x" in d and "last_y" in d:
                self._last_joints = (int(d["last_x"]), int(d["last_y"]))
        except Exception:
            pass

    def _save_frame(self, well=None):
        # Re-serialize the existing correction only -- never refit here (goto/close
        # call this on every move).
        d = dict(self.frame_correction)
        if self._last_joints is not None:
            d["last_x"], d["last_y"] = int(self._last_joints[0]), int(self._last_joints[1])
        if well:
            d["well"] = well.upper()
        if self._anchor_pts:
            d["anchors"] = sorted(self._anchor_pts)
        try:
            os.makedirs(os.path.dirname(self._frame_path), exist_ok=True)
            with open(self._frame_path, "w") as f:
                json.dump(d, f, indent=2)
        except Exception:
            pass

    def _check_frame(self):
        """On connect: report the position check (habit), flag a likely reset."""
        cur = self.joint_position()
        if self._last_joints is None:
            print(f"  position check: joints ({cur['X']},{cur['Y']}); no previous "
                  "pose on record. Run `check` to verify against A1.")
            return
        jump = abs(cur["X"] - self._last_joints[0]) + abs(cur["Y"] - self._last_joints[1])
        if jump > self.FRAME_RESET_THRESHOLD:
            self.frame_suspect = True
            print(f"  position check: ** MISMATCH ** now ({cur['X']},{cur['Y']}) vs "
                  f"last ({self._last_joints[0]},{self._last_joints[1]}), moved {jump} usteps.\n"
                  "  ** Likely a power-cycle or bump. Geometry is intact (NO re-teach):\n"
                  f"  ** jog the outlet over {self.ANCHOR_WELL} and run "
                  f"`reanchor {self.ANCHOR_WELL}` before goto.")
        else:
            print(f"  position check: OK — joints ({cur['X']},{cur['Y']}) match the "
                  f"last session ({jump} usteps drift). Frame looks intact.")

    ANCHOR_WELL = "A1"   # standard reference well for reanchor / check
    ANCHOR_WELLS = ("A1", "A12", "H1", "H12")   # 4 corners for the affine anchor

    def check(self, well_id: str = None):
        """Go to the anchor well so you can visually verify the calibration.

        Use after a suspected bump/power event: if the outlet is NOT centered on
        the well, run ``reanchor`` to recover (no re-teach needed).
        """
        well_id = (well_id or self.ANCHOR_WELL)
        self.goto_well(well_id)
        print(f"  CHECK: is the outlet centered on {well_id.upper()}?  "
              f"If yes, calibration is good. If not, jog onto it and run "
              f"`reanchor {well_id.upper()}`.")
        return self.joint_position()

    def add_anchor(self, well_id: str):
        """Capture ONE anchor: the live joints (you've jogged the outlet to center
        ``well_id``) vs the raw model prediction. Collect 4 corners, then fit_anchor().
        """
        self._require()
        if not (self.kin_model and self.kin_model.is_fitted):
            raise RuntimeError("no kinematic model fitted; nothing to anchor to")
        raw, _ = self._resolve_raw(well_id)          # uncorrected model prediction
        now = self.joint_position()                  # measured live joints
        w = well_id.strip().upper()
        self._anchor_pts[w] = ((now["X"], now["Y"]), (raw["X"], raw["Y"]))
        print(f"  anchored {w}: measured ({now['X']},{now['Y']}) vs "
              f"model ({raw['X']},{raw['Y']})  [{len(self._anchor_pts)} point(s); "
              f"make sure the outlet was centered]")
        return self._anchor_pts[w]

    def clear_anchors(self):
        self._anchor_pts = {}
        print("  anchor points cleared.")

    def _fit_correction(self):
        """Fit frame_correction from self._anchor_pts. >=3 -> full affine (clamped),
        else translation-only. Returns (correction, info_str)."""
        import numpy as np
        pts = list(self._anchor_pts.values())
        meas = np.array([m for m, _ in pts], float)
        mod = np.array([p for _, p in pts], float)
        if len(pts) == 0:
            return dict(IDENTITY_CORRECTION), "no anchors"
        # translation-only candidate (always valid)
        tx, ty = (meas - mod).mean(axis=0)
        trans = dict(IDENTITY_CORRECTION); trans["cx"] = float(tx); trans["cy"] = float(ty)
        if len(pts) < 3:
            return trans, f"translation-only ({len(pts)} pt)"
        # full 2D affine: [Xm,Ym,1] -> Xc ; [Xm,Ym,1] -> Yc
        M = np.column_stack([mod[:, 0], mod[:, 1], np.ones(len(pts))])
        (a, b, cx), *_ = np.linalg.lstsq(M, meas[:, 0], rcond=None)
        (d, e, cy), *_ = np.linalg.lstsq(M, meas[:, 1], rcond=None)
        aff = {"a": float(a), "b": float(b), "cx": float(cx),
               "d": float(d), "e": float(e), "cy": float(cy)}
        # --- guardrails: reject scale/shear or spans that smell like fitted noise ---
        lin_ok = (abs(a - 1) <= _MAX_LINEAR_DEV and abs(b) <= _MAX_LINEAR_DEV
                  and abs(d) <= _MAX_LINEAR_DEV and abs(e - 1) <= _MAX_LINEAR_DEV)
        # implied affine-vs-translation correction span across the taught wells
        span = 0.0
        try:
            allmod = np.array(
                [(p["X"], p["Y"]) for p in
                 (self.kin_model.predict(w, self.plate) for w in self.teach_table.taught)],
                float)
            ax = a * allmod[:, 0] + b * allmod[:, 1] + cx
            ay = d * allmod[:, 0] + e * allmod[:, 1] + cy
            tx2 = allmod[:, 0] + trans["cx"]; ty2 = allmod[:, 1] + trans["cy"]
            span = float(np.max(np.abs(ax - tx2)) + np.max(np.abs(ay - ty2)))
        except Exception:
            span = 0.0
        if not lin_ok or span > _MAX_SPAN_USTEPS * self._ustep_scale:
            return trans, (f"affine rejected (lin_ok={lin_ok}, span={span:.1f}) "
                           f"-> translation-only")
        # snap near-identity to exact identity
        if (abs(a - 1) < 1e-6 and abs(e - 1) < 1e-6 and abs(b) < 1e-6 and abs(d) < 1e-6
                and abs(cx) <= _SNAP_IDENTITY * self._ustep_scale
                and abs(cy) <= _SNAP_IDENTITY * self._ustep_scale):
            return dict(IDENTITY_CORRECTION), "snap-to-identity"
        return aff, f"full affine ({len(pts)} pts)"

    def fit_anchor(self):
        """Fit the joint-frame correction from the collected anchors and save it."""
        self._require()
        fc, info = self._fit_correction()
        self.frame_correction = fc
        # residual per anchor after correction
        res = []
        for w, ((mx, my), (xm, ym)) in self._anchor_pts.items():
            cx, cy = _apply_correction(fc, xm, ym)
            res.append((w, abs(cx - mx) + abs(cy - my)))
        now = self.joint_position()
        self._last_joints = (now["X"], now["Y"])
        self.frame_suspect = False
        self._save_frame(well=",".join(sorted(self._anchor_pts)))
        rs = ", ".join(f"{w}:{e}" for w, e in res)
        print(f"anchor fit [{info}] saved. residual usteps: {rs or '(none)'}")
        print(f"  correction: a={fc['a']:.4f} b={fc['b']:.4f} cx={fc['cx']:+.1f} | "
              f"d={fc['d']:.4f} e={fc['e']:.4f} cy={fc['cy']:+.1f}")
        return fc

    def reanchor(self, well_id: str = None):
        """One-well recovery after a power-cycle or bump (pure translation).

        Jog the outlet over ``well_id`` (default A1), then call this. Equivalent to
        ``add_anchor`` + ``fit_anchor`` with a single point. For a sharper edge fix,
        anchor the 4 corners (A1, A12, H1, H12) and fit_anchor().
        """
        well_id = well_id or self.ANCHOR_WELL
        self.clear_anchors()
        self.add_anchor(well_id)
        fc = self.fit_anchor()
        return (fc["cx"], fc["cy"])

    def fit_kinematics(self, n_starts: int = 400):
        """(Re)fit the 5-bar geometry from the taught wells and save it."""
        if KinematicModel is None:
            raise RuntimeError("scipy not available; cannot fit kinematics")
        m = KinematicModel()
        rms = m.fit(self.plate, self.teach_table, n_starts=n_starts)
        m.save()
        self.kin_model = m
        print(f"kinematics fitted & saved: RMS {rms:.2f} mm over "
              f"{len(self.teach_table.taught)} wells")
        return rms

    def goto_well(self, well_id: str, safe=True):
        """Move to a well: exact taught position, else derived from the metric map."""
        self._require()
        if self.frame_suspect:
            print("  ** frame looks power-cycle-reset — `reanchor <well>` first or "
                  "this move will be off (geometry is fine, no re-teach).")
        tgt, taught = self._resolve_well(well_id)
        travel_z = self.teach_table.travel_z() if safe else None
        print(f"goto {well_id.upper()} [{taught}] -> X={tgt['X']} Y={tgt['Y']} Z={tgt['Z']}")
        if travel_z is not None and self.teach_table.z_travel_usteps is not None:
            self._move_joints_to(z=travel_z)              # lift to safe travel height
            self._approach_joints(tgt["X"], tgt["Y"])     # creep in like the teach jogs
            self._move_joints_to(z=tgt["Z"])              # descend to the well
        else:
            self._approach_joints(tgt["X"], tgt["Y"])     # creep in like the teach jogs
            self._move_joints_to(z=tgt["Z"])              # set Z
        self._last_joints = (tgt["X"], tgt["Y"])          # checkpoint for frame-reset detection
        self._save_frame(well=well_id)
        return self.joint_position()

    def scan_wells(self, well_ids, dwell_s=0.0):
        """Visit a list of wells in order (e.g. a plate sweep)."""
        for wid in well_ids:
            self.goto_well(wid)
            if dwell_s:
                time.sleep(dwell_s)
