# Findings — hard-won facts about this Phil unit

Empirically established while bringing the arm up (2026-06-11). These are the
non-obvious things that cost the most time; read before debugging.

## Mechanism
- Phil is a **5-bar parallel arm** (two motors → two arms → meet at one outlet),
  **not** a Cartesian stage and **not** a microscope. X/Y are rotary joints with
  ~180° range; moving one motor swings the outlet on a **diagonal arc** (gains
  rows *and* columns), so single-motor jogs don't trace plate rows/columns.

## Controllers on USB
- `/dev/ttyACM1` = Teensy (Teensyduino, sn 16640550) → **this is Phil**.
- `/dev/ttyACM0` = Opentrons Flex → **unrelated**, ignore it.

## Firmware protocol mismatch (the big one)
- The repo's `control/microcontroller.py` assumes **8-byte commands / 24-byte
  status**. The flashed firmware is **older: 6-byte commands / 20-byte status**.
- Symptom with the stock driver: every command returns `CMD_CHECKSUM_ERROR`
  (status byte 2) and `wait_till_operation_is_completed` then calls
  `sys.exit(1)`. The CRC algorithm matches; the **framing** doesn't.
- We reverse-engineered it: 6-byte `[id, op, p0, p1, p2, crc8]`, 20-byte status
  `[id, status, X, Y, Z, theta, buttons, pad]`, CRC-8/CCITT, opcodes = repo
  `CMD_SET`. Implemented in `legacy_mc.py` — **no reflashing needed**. Use
  `backend="legacy"`.
- The status stream has **no delimiter**; the host must stay byte-aligned. The
  reader re-syncs on the echoed `cmd_id`.

## Open-loop, no encoders
- There is **no position feedback from manual motion**. Moving the arms by hand
  does **not** change the reported position (verified: it stayed fixed while
  being moved). So you can only know a position the controller *commanded*.
- Consequence: **teaching must be done with jog commands**, never by hand.
- Forcing the energized motors by hand can also make them **lose steps**
  (desyncs counter from physical) — avoid it.

## Frame persistence
- The firmware **keeps its position counter across host reconnects** (as long as
  the Teensy stays powered). So PhilRobot's legacy connect deliberately does
  **not** call `reset()`/`initialize_drivers()` — both of which **zero** the
  counter. Motion works without host init (the firmware self-inits at power-on).
- A **power-cycle** resets the counter → the absolute frame shifts by a constant
  joint offset. Recover with `reanchor(<well>)` (one well), not a re-teach. The
  shift is a pure per-axis translation, so one reference well fully recovers it.
- PhilRobot saves the last commanded pose (`phil_frame.json`) and on connect
  compares it to the live joints; a large jump => `frame_suspect` => it warns to
  reanchor before `goto`. So a crash/power loss is detected, never silent.

## Units / sensitivity
- 256 microstepping: commands in full-steps, position reported in microsteps
  (`V*256`). `legacy_mc` converts to repo usteps (÷8 out, ÷32 in).
- ~6 repo usteps/mm near the plate; **1 full-step (8 usteps) ≈ 1 mm**; ~50
  usteps per well. Early jogs of 300 usteps (~50 mm) repeatedly ran off the
  plate — **jog small**.

## Backlash
- Reversing direction loses ~1–2 full-steps of motion before the arm moves.
  Wiggling back-and-forth drifts and never returns cleanly. When teaching, make
  the **final approach in one direction** and accept "over the well" (no need to
  dead-center). The model averages it out.
- A jog step that isn't a multiple of 8 usteps rounds (e.g. 3 → 0 = no move).
  Jog sizes are kept to multiples of 8.

## Calibration models (what we tried, in order)
1. **Affine** (plate mm → joints): fails — the 5-bar is too curved (4 corners
   don't form a parallelogram in joint space; ~150-ustep error). Note: 3 points
   fit an affine with 0 residual = false confidence; the 4th reveals the error.
2. **RBF curve-fit** (`well_map.py`): ~2–3 mm typical, but **sags in sparse
   regions** (e.g. B10 landed half a well off). Good fallback, not the answer.
3. **5-bar kinematic model** (`kinematics.py`): **the solution.** Fit from ~10
   spread wells → RMS ≈ 0.2 mm; computes every well/every plate from geometry,
   uniform accuracy. Verified: untaught B10 and G9 land on target.

## Precision ceiling
- ~1–2 mm, set by open-loop steppers + backlash. The kinematic model itself is
  sub-millimeter; the remaining error is mechanical. Adding encoders or
  backlash-compensated approaches would be the next lever.
