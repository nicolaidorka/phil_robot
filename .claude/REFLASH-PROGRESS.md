# Phil microstep reflash — PROGRESS & hardware facts (travels with the repo)

Active execution log for the microstep reflash (companion to the background/plan
in [FUTURE-microstep-reflash.md](FUTURE-microstep-reflash.md)). Everything here is
committed so it survives sessions and machines. **Decision: GO** (2026-06-12).

## Why we're doing it (one paragraph)
Phil's positioning is coarse because the *current custom firmware only accepts
whole full-step move commands*. On this 5-bar arm one full-step swings the tip
**1.9–5.0 mm** (median 3.4, worst on the right/high columns) — larger than both
the ~1 mm backlash and the 0.5 mm model error, so a better well-map cannot help.
Only finer command resolution can. The octopi v2 firmware commands in **microsteps**,
which removes that grid. Honest expected outcome: **3–5 mm → ~0.5–1 mm** (the
open-loop backlash floor; no encoders).

## HARDWARE — confirmed by eye/camera 2026-06-12 (do not re-verify)
- **Controller**: Squid/octopi **V4** main board (silkscreen "V4").
- **MCU**: **Teensy 4.1** (processor marked `MIMXRT1062DVJ6B` = i.MX RT1062).
- **Stepper driver**: **TMC2660C** (read `TMC2660C 2332 BCK27`) on plug-in modules.
- **Motion controller**: **TMC4361A** (read `TMC4361ALA 2202A039K`).
- **Motors**: Shinano Kenshi **SST43D1125** (NEMA-17 frame), lot `09520G`.
- **Teensy USB serial**: `/dev/ttyACM0`, SN `16640550`, mfr "Teensyduino".
=> The board is exactly what `octopi_firmware_v2/main_controller_teensy41` targets
   (`tmc4361A_tmc2660_config`). **v2 firmware is compatible.** Compile target = Teensy 4.1.

## v2 SERIAL PROTOCOL (verified in the firmware .ino @ e566e01^)
Main command link = plain `SerialUSB` @ 2,000,000 baud (NOT PacketSerial; that's
only the joystick on Serial5). Fixed frames, no delimiter:
- **command** = 8 bytes `[cmd_id, opcode, p0,p1,p2,p3, p4, crc8]`; positions are a
  4-byte **signed big-endian** value in p0..p3, **in MICROSTEPS** (no full-step rounding).
- **status** = 24 bytes `[cmd_id, status, X(4 BE), Y(4 BE), Z(4 BE), THETA(4 BE), …,
  byte18 = switches/joystick, …]`.
- **CRC** = CRC-8/CCITT over the first 7 bytes (same table as legacy/crc8.cpp).
- **opcodes**: MOVE_X/Y/Z=0/1/2, HOME_OR_ZERO=5, MOVETO_X/Y/Z=6/7/8,
  SET_AXIS_DISABLE_ENABLE=32, INITIALIZE=254, RESET=255.
- **HOME_OR_ZERO payload[1]**: HOME_POSITIVE=0, HOME_NEGATIVE=1, **HOME_OR_ZERO_ZERO=2**
  (zero current pos). NOTE: differs from the legacy driver's `=3`.

## DONE so far (all reversible — nothing flashed; robot untouched)
- [x] Gate 1 (board compatibility) — CLEARED (above).
- [x] Calibration snapshot → `software/phil/config/pre-reflash-backup/` (+ RESTORE_NOTES.txt).
      Legacy rollback restores these 4 .json; they do NOT carry to v2 (scale changes → re-teach).
- [x] v2 firmware restored from git into `firmware/octopi_firmware_v2/main_controller_teensy41/`.
- [x] **`def_phil.h`** authored (joystick off, 256 microstepping, LOW current 500/500/300 mA,
      reduced vel/accel for bring-up). Must be wired by uncommenting `#include "def_phil.h"`
      in the .ino (lines 8–13, currently `def_octopi_80120.h`).
- [x] **`software/phil/hardware/v2_mc.py`** driver written (8/24/4-byte protocol, microstep
      commands, mirrors legacy_mc.py interface). Imports CRC + port-discovery from legacy_mc.
- [x] Camera-guided inspection tooling → `.claude/tools/capframe.py`, `crop.py`.

## TODO (remaining)
- [ ] Wire `backend="v2"` in `software/phil/robot.py` to construct `V2Microcontroller`.
- [ ] Rescale python constants for the finer unit: `constants.MICROSTEPPING_*`,
      `jog_teach.STEPS`, `robot.FRAME_RESET_THRESHOLD`, affine span; un-hardcode
      `jog_teach.py` `backend="legacy"`; clear/ignore old `phil_frame.json`.
- [ ] Toolchain: install Teensy core (Arduino IDE 2.3.x AppImages already in ~/ and
      ~/Downloads; need FastLED + PacketSerial libs) and **compile `.hex` WITHOUT uploading**.
- [ ] Optional Gate 2: find the *current custom* firmware `.ino` off-machine for a
      byte-identical rollback (not blocking — v2-from-repo is a guaranteed-working floor).
- [ ] **FLASH** (user): udev rule → Teensy Loader → load `.hex` → PROGRAM button.
- [ ] Bring-up: zero via HOME_OR_ZERO_ZERO (no homing); tiny one-axis move; confirm
      direction/scale; **watch motor temperature**; then re-teach + re-fit + verify.

## Agent review findings (2026-06-12) — 3 reviewers: driver / config / integration
FIXED in this state:
- [x] **Compile blocker**: def_phil.h was missing `R_sense_xy`/`R_sense_z` (firmware current calc
      needs them). Added standard Squid V4 values 0.22 / 0.43 — VERIFY against the board before flash.
- [x] **Polarity comment bug**: v2_mc.py `set_axis_enable_disable` comment had enable/disable backwards.
      Firmware truth: status==0 disables, !=0 enables. Comment corrected; behavior is transparent pass-through.

MUST FIX before flashing (still open — all in the TODO):
- [ ] **`.ino` include still selects `def_octopi_80120.h`** (line 9), NOT def_phil.h. Flashing as-is would
      load the wrong stage's current/joystick. Comment line 9, add `#include "def_phil.h"`. (CRITICAL)
- [ ] **backend="v2" not wired** in robot.py — it currently falls through to the stock Qt driver. Add an
      `elif backend=="v2"` branch building V2Microcontroller, with a connect path that does ONE
      `initialize_drivers()` (applies def_phil.h current/ustep/ramp) and does NOT auto-`reset()`. (CRITICAL)
- [ ] **Count constants ~32x too small** at the new microstep scale (~175 microsteps/mm at the tip vs legacy
      ~5.5/mm). Rescale by 32: robot.py MOVE_CHUNK_USTEPS(40→~1280), APPROACH_PRE_USTEPS(80→~2560),
      APPROACH_CONFIRM_TOL & OK(8→~256), APPROACH_MAX_CORRECTION(30→~960), FRAME_RESET_THRESHOLD(80→~2560),
      _MAX_SPAN_USTEPS(12→~384), the line-564 noise gate(16→~512); jog_teach.STEPS → ~[64,128,256,512,1024];
      set constants.MICROSTEPPING_*=256 (fixes the real Z mm-path too). _MAX_LINEAR_DEV is dimensionless—leave.
      Best done as a single SCALE/USTEPS_PER_MM source shared by both backends. (CRITICAL)
- [ ] **Toolchain**: FastLED + PacketSerial libs not installed; needed to compile (PacketSerial is included
      unconditionally even with joystick off). Install, then COMPILE .hex without flashing to prove it builds. (WARN)
- [ ] un-hardcode `jog_teach.py` backend="legacy".

NOTED (not blocking bring-up):
- X/Y current scale lands at CS=14 (firmware comment prefers >16); harmless, bump X/Y to ~570mA only if weak.
- v2_mc set_max_velocity_acceleration is a no-op (relies on def_phil.h ramps) → CLI set_speed/apply_motion_profile
  are inert on v2; fine for bring-up.
- THETA status bytes [14-17] are never written by firmware → theta_pos is garbage (unused; ignore).
- robot.py home_arms() polarity must be corrected before any v2 limit-switch homing (we avoid homing at bring-up).

## Rollback
- Legacy `.ino` found → flash it, set `backend="legacy"`, restore the 4 backup .json. Byte-identical.
- Not found → stay on v2-from-repo + recalibrate (always available). A Teensy can't be bricked
  (PROGRAM-button mass-erase always re-flashes).
