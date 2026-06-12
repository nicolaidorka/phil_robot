# PARKED: Reflash Phil's Teensy to microstep-capable firmware

**Status: not being done now.** This is a saved reference. Do NOT flash anything based on this without
first clearing the two prerequisites below. Nothing here touches the robot.

## Goal & why parked
Goal would be finer well-to-well positioning. Phil's *current* firmware only accepts whole **full-step**
move commands (~1.5 mm grid); the repo's v2 firmware commands in **microsteps**. Parked because:
- **No rollback firmware exists.** The running firmware is a *custom* build (6-byte cmd / 20-byte status /
  3-byte positions / 256 microstepping / opcodes MOVETO=6, HOME=5). It matches **no** octopi-research
  release (upstream went 4-byte → 8-byte; there was never a 6-byte version) and is **not on this machine**
  (searched). A Teensy 4.x **cannot have its flash read back**, so today's exact firmware is
  **unrecoverable** unless the original `.ino` is found off-machine (whoever built Phil / another lab PC /
  a USB/Drive).
- **Gain is capped by mechanics.** Microstepping removes the ~1.5 mm full-step *command* grid, but Phil is
  open-loop with **~1 mm backlash** microstepping can't fix. Realistic: ~1.5 mm → **~0.5–1 mm**, not 0.2 mm.
- It's a multi-part project with real hardware risk (wrong motor current can cook a motor).

## Two PREREQUISITES before revisiting
1. **Confirm the controller board by reading the chip labels** (the "256 microstepping" clue is NOT proof —
   TMC2209 also does 256). Need **TMC4361A + TMC2660** (v2 fits) vs **TMC2209** (v2 will NOT drive the
   motors — different plan). Confirm the Teensy is a **4.1**.
2. **Find the legacy firmware `.ino`** off-machine for a true rollback (ask whoever built Phil; check other
   lab PCs / USB / Drive).

## Corrected technical findings (from multi-agent review)
- **v2 firmware** (`firmware/octopi_firmware_v2/main_controller_teensy41`, Teensy 4.1, TMC4361A/TMC2660)
  commands in microsteps. Recoverable from THIS repo's git history at **`e566e01^`** — a guaranteed-available
  fallback `.hex`, but reverting to it = v2 + recalibration, NOT byte-identical to today.
- **A Teensy is never truly bricked** (PROGRAM-button mass-erase always re-flashes). Real risk = downtime +
  needing a known-good `.hex` + toolchain. Toolchain not installed (Arduino IDE AppImage present, no Teensy core).
- **Do NOT use `backend="stock"`** — `control._def` `sys.exit(1)`s at import with no `configuration*.ini`,
  imports Qt, and waits `sys.exit` on timeout. Instead write a new **`phil/hardware/v2_mc.py`** mirroring
  `legacy_mc.py` (8-byte cmd / 24-byte status / 4-byte microstep positions; reuse its crc8 + reader). Add `backend="v2"`.
- **Scale change breaks calibration + constants.** Re-teach + re-fit; change `constants.MICROSTEPPING_*`,
  rescale `jog_teach.STEPS`, `FRAME_RESET_THRESHOLD`, the affine `_MAX_SPAN_USTEPS`; clear `config/phil_frame.json`;
  un-hardcode `jog_teach.py` `backend="legacy"`.

## Safe sequence when revisited
1. **Snapshot calibration first**: git-tag `phil_teach.json` + `phil_kinematics.json` + `phil_calibration.json`;
   copy gitignored `phil_frame.json` aside; record `(LEGACY=256, REPO=8, CMD_DIVISOR=32)` and Teensy SN `16640550`.
2. Install Teensy toolchain.
3. Recover v2 firmware from git history; author a safe `def_phil.h` (real motor current — start LOW;
   `ENABLE_JOYSTICK=false`; StallGuard off; no homing). **Compile both candidate `.hex` WITHOUT uploading**
   so the first flash is already reversible.
4. Write `phil/hardware/v2_mc.py` + `backend="v2"`.
5. **User flashes** (first = v2). Zero with HOME_OR_ZERO_ZERO (no homing). Smoke-test ONE axis: tiny move,
   confirm direction/scale, **watch motor temperature**.
6. Recalibrate (constants + re-teach + re-fit + re-anchor).
7. Verify with a dial indicator; expect ~0.5–1 mm (backlash floor).
8. **Rollback** = flash legacy `.hex` (if found) → `backend="legacy"` + restore tagged calibration; else
   flash v2 → `backend="v2"` + recalibrate.

## Smaller win available WITHOUT reflashing
The affine `anchor` is in place. Backlash is direction-dependent — `goto_well` already does
lift→swing→descend; making the final XY approach always come from the same direction could shave the
backlash component. Low-risk, software-only.

## Key files for whoever picks this up
- `software/phil/hardware/legacy_mc.py` (template for `v2_mc.py`; documents the current protocol)
- `software/phil/robot.py` (backends; scale-coupled `FRAME_RESET_THRESHOLD`, anchor guards)
- `software/phil/constants.py` (`MICROSTEPPING_*` — scale-coupled)
- `software/phil/jog_teach.py` (hardcoded `backend="legacy"`; `STEPS`)
- `firmware/octopi_firmware_v2/main_controller_teensy41/` (recover from git `e566e01^`; `def_*.h` configs)
- Calibration: `config/phil_teach.json`, `phil_kinematics.json`, `phil_calibration.json`, `phil_frame.json`
