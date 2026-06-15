# Architecture — Phil robot control (`software/phil/`)

## Layers (top to bottom)

```
cli.py  jog_teach.py  selftest.py  rehome.py  measure.py  drive.py   user surfaces (root)
        |
PhilRobot (robot.py)              high-level: connect, jog, goto_well, teach, reanchor
        |        \
  well resolution  motion
        |              \
geometry.kinematics   hardware.legacy_mc / hardware.v2_mc   (backend-selected driver)
geometry.well_map / geometry.calibration    |
geometry.teach / geometry.well_plate     Teensy firmware over USB serial
```

Package layout: `robot.py` + `paths.py` + `constants.py` and the entry points at the
top; well->joint models in `geometry/` (well_plate, teach, calibration, kinematics,
well_map); drivers in `hardware/` (legacy_mc, v2_mc). All data-file paths resolve
through `phil/paths.py` (so modules can live in subpackages while `config/` and
`labware/` stay at the package root).

**Entry points (each `python3 -m phil.<name>`):**

| module | what it's for |
|--------|---------------|
| `cli.py`       | interactive shell: jog, teach, `goto`, `sweep`, `metric`, … (main surface) |
| `jog_teach.py` | arrow-key teach console with auto-approach (`--all` walks every well) |
| `drive.py`     | free arrow-key drive — just move the arm, **no teaching** |
| `rehome.py`    | the ONE blessed recovery: jog onto true A1 centre → set home → confirm |
| `measure.py`   | drive each well via the *model*, jog to true centre, record the offset |
| `tiptrack.py`  | **live GUI**: a plate-grid window with a marker on the converged tip (`KinematicModel.forward`) that follows the arm as you jog — positioning becomes *observable* instead of blind; also teaches (Enter) / `fitkin` (`f`). Sim-safe (throwaway config). |
| `stepcheck.py` | flags **mis-taught wells**: in joint space each taught well must differ from its grid-neighbours by a predictable (smoothly-varying) step delta; `--tol-mm` sets the flag threshold. Labware-specific. |
| `selftest.py`  | connection + feedback + tiny-jog hardware check |

Shared helper (not an entry point): **`viz.py`** — backend-agnostic matplotlib
patch/collection helpers for the plate visualizations (`tiptrack`, the
model-vs-grid overlay). Imports no `pyplot`, so callers pick the backend.

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
   The production *strategy* is to teach the **boundary** (rows A–E and H, 72/96)
   so step 2 only ever *interpolates* the untaught interior (F/G). **⚠️ Current
   state ≠ strategy: only 24/96 are taught in the v2 frame** (L-shape: row A +
   col 1 + a few); the proven 72-well boundary teach is **pre-reflash and invalid
   in v2** and hasn't been rebuilt — so step 2 below is doing heavy *extrapolation*
   today. `jog_teach --all --v2` rebuilds it.
2. **`KinematicModel`** (`kinematics.py`) — inverse kinematics of the 5-bar.
   With the boundary taught it *interpolates* the interior (F6 verified ~0.5 mm);
   **with only the current 24-well L-shape it *extrapolates* — corner LOO ≈ 12–13
   mm.** It extrapolates poorly in general (the early ~10-well fit had edge LOO
   1.5–4.3 mm), column 1 is the weak edge — which is why the boundary must be
   taught. Also the path for
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
  `soft_l1` loss then a `linear` polish, with a geometry soft-prior pulling links
  toward 65/145 mm. Fed the taught `(plate-mm ↔ joint)` pairs. The two arms come
  out near-real (proximal ~65/59 mm, distal ~141/149 mm). **Current fit (v2): the
  24 taught wells → RMS ≈ 0.93 mm in-sample.** ⚠️ The often-quoted **all-72-wells,
  RMS ≈ 0.42 mm fit was PRE-REFLASH** and is invalid in v2 units. (An early ~10-well
  fit hit RMS ≈ 0.2 mm but overfit — edge LOO ≈ 1.5–4.3 mm. Fitting the dense
  boundary trades a little in-sample RMS for far better interior interpolation —
  F6 was verified ~0.5 mm **back then, on the 72-well legacy fit**; that no longer
  holds under the current 24-well v2 fit.)
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
uses, so PhilRobot is backend-agnostic (`backend="legacy"|"v2"|"stock"|"sim"`).

**`v2_mc.py`** drives the newer **microstep firmware** (the reflash effort — see
[REFLASH-PROGRESS](REFLASH-PROGRESS.md), [V2-FIRMWARE-NOTES](V2-FIRMWARE-NOTES.md)).
It reports/commands at a much finer scale than legacy (~175 counts/mm at the tip
vs legacy ~5.5), so PhilRobot rescales every count-based tunable by
`_ustep_scale` (32 for v2, 1 for legacy) to keep motion logic backend-agnostic.
A saved frame records which scale it was written in (256 = v2, 8 = legacy) so a
frame is never mis-read across firmware. v2 goto fixes: [V2-GOTO-FIXES](V2-GOTO-FIXES.md).

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
| `phil_frame.json`      | reanchor offset + last pose (power-cycle detect) + `ustep_scale` tag | `goto`, `reanchor`, close |
| `phil_offsets.json`    | per-well model-vs-true offsets (diagnostic) | `measure` — created on first run; absent until then |

The active `phil_teach.json` is also backed up under `config/_GOOD_BACKUP/`, and
the pre-reflash **legacy-units** teach/kinematics live in `config/pre-reflash-backup/`
(72 wells — **not** valid in the v2 frame; kept for reference only).
