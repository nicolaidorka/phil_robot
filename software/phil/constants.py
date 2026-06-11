"""Physical / motion constants for the Phil robot.

These mirror the values the Squid firmware is configured with in
``software/control/_def.py`` so the ``phil`` package can do mm<->microstep
math and provide a sensible out-of-the-box (uncalibrated) well model
*without* importing the heavyweight microscope ``control._def`` module
(which requires a configuration*.ini on disk and pulls in Qt).

If you change the stepper/leadscrew configuration in ``_def.py``, update
these to match.
"""

# --- axis identifiers (match control._def.AXIS) -----------------------------
AXIS_X = 0
AXIS_Y = 1
AXIS_Z = 2
AXIS_XY = 4

# --- leadscrew / stepper geometry (match control._def.py) -------------------
SCREW_PITCH_X_MM = 1.0
SCREW_PITCH_Y_MM = 1.0
SCREW_PITCH_Z_MM = 0.012 * 25.4          # 0.3048 mm

FULLSTEPS_PER_REV_X = 200
FULLSTEPS_PER_REV_Y = 200
FULLSTEPS_PER_REV_Z = 200

MICROSTEPPING_X = 8
MICROSTEPPING_Y = 8
MICROSTEPPING_Z = 8

# direction the firmware applies to absolute moves (control._def.STAGE_MOVEMENT_SIGN_*)
STAGE_MOVEMENT_SIGN_X = -1
STAGE_MOVEMENT_SIGN_Y = 1
STAGE_MOVEMENT_SIGN_Z = -1

# --- default motion profile (mm/s, mm/s^2) ----------------------------------
# Conservative defaults for smooth arm travel; tune via PhilRobot or the CLI.
# Firmware caps: velocity <= 655.35 mm/s, acceleration <= 6553.5 mm/s^2.
DEFAULT_MAX_VELOCITY = {"X": 15.0, "Y": 15.0, "Z": 2.0}
DEFAULT_MAX_ACCELERATION = {"X": 200.0, "Y": 200.0, "Z": 20.0}

# --- Phil's native well grid convention (control._def.PLATE_READER) ---------
# Used to build the nominal (uncalibrated) plate-local -> robot transform.
PLATE_READER_OFFSET_COLUMN_1_MM = 20.0   # robot X of column 1
PLATE_READER_OFFSET_ROW_A_MM = 20.0      # robot Y of row A
PLATE_READER_SPACING_MM = 9.0

# Z convention: in the robot mm frame, +Z is UP (arm lifted / away from the
# plate), -Z / smaller Z is DOWN (toward the well). So the safe travel height
# is MORE positive than the working height. Placeholders until calibrated.
DEFAULT_Z_WORKING_MM = 2.0     # descend to this at a well
DEFAULT_Z_SAFE_MM = 12.0       # lift to this for XY travel between wells

# Soft travel limits in robot mm (prevent crashes). Generous defaults; tune.
SOFT_LIMITS_MM = {
    "X": (-1.0, 130.0),
    "Y": (-1.0, 130.0),
    "Z": (-1.0, 40.0),
}


def mm_per_ustep(axis: str) -> float:
    """Millimetres travelled per microstep for the given axis ('X'/'Y'/'Z')."""
    pitch = {"X": SCREW_PITCH_X_MM, "Y": SCREW_PITCH_Y_MM, "Z": SCREW_PITCH_Z_MM}[axis]
    micro = {"X": MICROSTEPPING_X, "Y": MICROSTEPPING_Y, "Z": MICROSTEPPING_Z}[axis]
    fullsteps = {"X": FULLSTEPS_PER_REV_X, "Y": FULLSTEPS_PER_REV_Y, "Z": FULLSTEPS_PER_REV_Z}[axis]
    return pitch / (micro * fullsteps)


def mm_to_usteps(axis: str, mm: float) -> int:
    """Signed microsteps for an absolute target, including stage direction sign."""
    sign = {"X": STAGE_MOVEMENT_SIGN_X, "Y": STAGE_MOVEMENT_SIGN_Y, "Z": STAGE_MOVEMENT_SIGN_Z}[axis]
    return int(round(sign * mm / mm_per_ustep(axis)))


def usteps_to_mm(axis: str, usteps: int) -> float:
    """Inverse of :func:`mm_to_usteps` (firmware position readout -> mm)."""
    sign = {"X": STAGE_MOVEMENT_SIGN_X, "Y": STAGE_MOVEMENT_SIGN_Y, "Z": STAGE_MOVEMENT_SIGN_Z}[axis]
    return usteps * sign * mm_per_ustep(axis)
