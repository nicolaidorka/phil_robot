# Architecture — Phil robot control (`software/phil/`)

## Layers (top to bottom)

```
cli.py / jog_teach.py / selftest.py   user surfaces (entry points at root)
        |
PhilRobot (robot.py)              high-level: connect, jog, goto_well, teach, reanchor
        |        \
  well resolution  motion
        |              \
geometry.kinematics   hardware.legacy_mc (LegacyMicrocontroller)
geometry.well_map / geometry.calibration    |
geometry.teach / geometry.well_plate     Teensy firmware over USB serial
```

Package layout: `robot.py` + `paths.py` + `constants.py` and the three entry points
(`cli.py`, `jog_teach.py`, `selftest.py`) at the top; well->joint models in
`geometry/` (well_plate, teach, calibration, kinematics, well_map); driver in
`hardware/` (legacy_mc). All data-file paths resolve through `phil/paths.py` (so modules
can live in subpackages while `config/` and `labware/` stay at the package root).

## The mechanism

Phil is a **5-bar parallel manipulator**: two base motors, each rotates a
proximal link; a distal link hangs off each; the two distal links meet at the
outlet (end-effector). So the outlet (x, y) is a closed-form function of the two
motor angles, and vice-versa.

- Joint **X** = one arm motor, joint **Y** = the other, joint **Z** = vertical.
- We read/command motor positions in "repo usteps" (see units below). The
  physical outlet position is recovered via the kinematic model.

## Well resolution (`PhilRobot._resolve_well`)

Order of preference for a well's joint target:

1. **Exact taught** (`teach.py` `TeachTable`) — the recorded joints for a well.
   **This always wins when present.** Measured ground truth beats any model.
   The production strategy: teach the **boundary** (rows A–E and H, 72/96), which
   brackets the only untaught wells — interior rows **F and G** — so step 2 only
   ever *interpolates* them. `jog_teach --all` teaches more wells if wanted.
2. **`KinematicModel`** (`kinematics.py`) — inverse kinematics of the 5-bar,
   refit on all 72 taught wells. Used for the interior F/G wells: bracketed by
   taught rows, it interpolates them (F6 verified ~0.5 mm). It *extrapolates*
   poorly (the original ~10-well fit had edge LOO ≈ 1.5–4.3 mm) — column 1 is
   still weak — which is exactly why the boundary is taught. Also the path for
   any other labware (no taught wells yet).
3. **`WellMap`** (`well_map.py`) — scipy RBF interpolation of taught wells.
   Sags in sparse regions.
4. **`Calibration`** (`calibration.py`) — affine (plate mm → joints). Coarse;
   only adequate for a true Cartesian stage, kept for completeness.

A persisted **joint-frame offset** (`phil_frame.json`, set by `reanchor`) is
added to the result so the calibration survives a power-cycle.

## Kinematic model (`kinematics.py`)

Parameters (12), all lengths in plate-local mm, angles in radians:
`base1(x,y), base2(x,y), l1, dist1, l2, dist2, s1, o1, s2, o2`, where
`theta_i = s_i * joint_i + o_i` (ustep → angle) and
`elbow_i = base_i + l_i·(cos θ_i, sin θ_i)`, outlet `E` = circle-intersection of
`(elbow_1, dist_1)` and `(elbow_2, dist_2)`. Z is a small tilt plane
`Z = az·x + bz·y + cz`.

- **Fit**: `scipy.least_squares` multistart (random restarts, two FK branches),
  `soft_l1` loss then a `linear` polish. Fed the taught `(plate-mm ↔ joint)`
  pairs. The two arms come out near-identical (proximal ~64 mm, distal ~145 mm,
  pivots ~41 mm apart) — a real mechanism. **Current fit: all 72 taught
  (boundary) wells, ~300 starts → RMS ≈ 0.42 mm in-sample.** (An early ~10-well
  fit hit RMS ≈ 0.2 mm but overfit — edge LOO ≈ 1.5–4.3 mm. Fitting the dense
  boundary trades a little in-sample RMS for far better interior interpolation:
  F6 verified ~0.5 mm.)
- **Inverse** (`predict`): well mm → joints via two circle intersections
  (elbow = circle(base, l) ∩ circle(E, dist)), then `joint = (θ - o)/s`, with
  angle-wrap unwrapping into the plausible joint range.
- Saved to `config/phil_kinematics.json`. Loaded on `PhilRobot.__init__`.
- **Caveat — overfits.** The in-sample RMS is sub-mm, but leave-one-out over the
  taught wells is ≈ 1.5 mm avg / 4.3 mm worst at the edges: 12 params soak up
  per-well mechanical error (backlash, flex) instead of true geometry. So the
  model is a **fallback for untaught wells only**; taught wells are replayed
  exactly (`_resolve_well` step 1). The production strategy is teach-every-well.

## Driver (`legacy_mc.py`)

The flashed Teensy runs an **older protocol** than the repo's
`control/microcontroller.py` expects (which does NOT work — see FINDINGS):

- Command = **6 bytes**: `[cmd_id, opcode, p0, p1, p2, crc8]`
- Status  = **20 bytes**: `[cmd_id, status, X(4 BE), Y(4 BE), Z(4 BE),
  theta(4 BE), buttons, pad]` (no trailing CRC)
- CRC-8/CCITT (poly 0x07); opcodes match `control._def.CMD_SET` (MOVE_X=0,
  MOVETO_X=6, HOME_OR_ZERO=5, SET_MAX_VELOCITY_ACCELERATION=22, INITIALIZE=254,
  RESET=255).
- A background thread keeps byte-alignment on the delimiter-less 20-byte stream
  using the echoed `cmd_id`.

`LegacyMicrocontroller` mimics the subset of `Microcontroller` that PhilRobot
uses, so PhilRobot is backend-agnostic (`backend="legacy"|"stock"|"sim"`).

## Units

- Firmware uses **256 microstepping**: a relative MOVE of value V changes the
  reported position by `V*256`. Commands are in **full-steps**, position is
  reported in **microsteps**.
- `legacy_mc.py` converts at its boundary to "repo usteps" (microstepping 8,
  the rest of the package's convention): command full-steps = repo_usteps / 8;
  reported repo_usteps = firmware_microsteps / 32.
- Physically near the plate: ~6 repo usteps / mm, **1 full-step ≈ 1 mm** of
  outlet travel, **~50 usteps per 9 mm well**. (These are local — the rotary
  arm is nonlinear; trust the kinematic model, not a single scale.)

## Labware (`well_plate.py`)

Opentrons-schema JSON. `WellPlate.load(name_or_path)` resolves by name from the
single `labware/` folder (all plate JSON lives there). `local_xy(well)` gives plate-local mm. The
default is the Eppendorf twin.tec LoBind 96 PCR (the plate physically on Phil).
Switching plates = different JSON; the kinematics maps its mm → joints.

## Config files (`config/`)

| file | what | changes when |
|------|------|--------------|
| `phil_kinematics.json` | fitted 5-bar geometry | re-teach + `fitkin` (rare) |
| `phil_teach.json`      | taught well joints   | teaching |
| `phil_calibration.json`| affine reference pts | teaching |
| `phil_frame.json`      | reanchor offset + last pose (power-cycle detect) | `goto`, `reanchor`, close |
