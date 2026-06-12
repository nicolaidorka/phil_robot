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
3. **5-bar kinematic model, fit from ~10 spread wells**: looked like the
   solution — in-sample RMS ≈ 0.2 mm. **But it overfits.** Leave-one-out over
   those wells is ≈ **1.5 mm avg, 4.3 mm worst at the edges**: 12 params absorb
   per-well mechanical error (backlash, flex) rather than true geometry, and
   with only ~10 points the model must *extrapolate* to the edges, where it's
   worst.
4. **Teach the boundary, refit, interpolate the interior** — the production
   strategy, and what fixes (3). Teach **rows A–E and H** (72/96; all 12 columns
   of each). Those rows *bracket* the only untaught wells — interior **rows F and
   G** — so the refit model **interpolates** F/G between taught neighbors instead
   of extrapolating to an edge. Refit the 5-bar on all 72 (`fitkin`, ~300 starts
   → **RMS ≈ 0.42 mm** in-sample). `_resolve_well` is **taught-first**: the 72
   boundary wells replay their exact recorded joints; F/G come from the refit
   model. **Verified: F6 lands ~0.5 mm via the model.** Caveat: **column-1
   (the far edge) is still weak** — even bracketed, the edge column is the
   shakiest; teach those wells (or accept ~1 mm) if it matters.

## Precision ceiling
- ~1–2 mm, set by open-loop steppers + backlash. A taught well replays its exact
  joints but still lands within backlash of where it was taught — that's the
  floor. The refit model adds ~0.5 mm of interpolation error on top for the
  interior F/G wells. Confirmed on hardware: a full-plate goto sweep lands the
  end piece ~1–1.5 mm off **even on taught wells** (the two arms don't converge
  exactly over the well). Decomposition: ~0.5 mm full-step command quantization
  (`legacy_mc._send_pos` sends whole full-steps) + ~1 mm backlash (dominant).

### Anti-backlash approach — TRIED, made it WORSE, reverted (2026-06-11)
- **What we tried:** a uniform fixed-direction final approach in `goto_well` —
  pre-position 16 usteps (2 full-steps) short of the XY target and come in from
  a fixed **+X/+Y** direction, so backlash is always taken up the same way.
  (`_approach_joints` helper + `BACKLASH_TAKEUP_USTEPS` const.)
- **Result:** slightly **worse**, not better, by eye. Reverted (not committed).
- **Why it failed:** wells were **taught in snake order** (row A L→R, row B R→L,
  …), and a plain `goto` sweep is *also* snake order — so the original
  straight-in-from-previous-well move already arrives at most wells from
  **roughly the direction they were taught**, and backlash mostly cancels by
  luck of matching order. Forcing a *uniform* +X/+Y direction matched the L→R
  rows but **opposed** the R→L rows, making those ~1 full-step worse. A single
  global approach direction is wrong for snake-taught data.
- **What it proved:** backlash **direction is a real, controllable lever** — the
  uniform approach visibly changed where the arm landed. So the idea isn't dead,
  just mis-applied.

### Ideas not yet tried (next levers, in rough order of promise)
0. **Mimic the teach motion (lead candidate).** Teaching nudges with **relative**
   jogs (`MOVE_X`, small steps `[8,16,32,64,120]` usteps) and the recorded count
   is "where the joint sits after small nudges in your final direction." But
   `goto` replays with a single **absolute** `MOVETO_X` from wherever the arm was
   — same count, *different backlash/step state*. Suspect this is the core
   replay error. Fix to try: make `goto`'s **final approach do a few small
   relative jog-steps in a fixed direction** (imitate the arrow keys) instead of
   one absolute slam, so the joints settle in the same state as when taught.
   Distinct from the failed uniform anti-backlash: that guessed a global
   direction with one absolute move; this reproduces the *motion*, not just the
   endpoint. (User noticed the teach arrow keys move in small steps — that's the
   tell.)
1. **Direction-matched approach:** approach each well from the **same direction
   it was taught** — snake-aware (+X on L→R rows, −X on R→L rows), or better,
   **record the actual final approach direction per well at teach time** and
   replay it. This *cancels* backlash instead of fighting it.
2. **Characterize the motion empirically ("learn how we move them"):** measure
   commanded-vs-physical offset per joint per direction (by eye/camera over a
   grid), build a small per-direction backlash-correction lookup, and apply it
   in `goto`. Turns the hand-wavy ~1 mm into a measured, compensable model.
3. **Smoother motion / settle:** slower final-leg velocity or a short settle
   dwell before reading/using position, to cut end-of-move overshoot (we saw a
   consistent +1 full-step end-of-move counter step).
4. **Hardware:** the only sub-mm fixes are the **firmware reflash** (parked —
   commands in microsteps, kills the quantization term) and/or **encoders**
   (closes the loop, kills backlash). Both are bigger projects.
- NOTE for evaluating any future fix: the firmware **readback is not
  trustworthy** at ustep resolution (quantized to 8 usteps, direction-biased,
  sampled before settle). Judge by **eye/camera on the well**, never by the
  reported joints.
