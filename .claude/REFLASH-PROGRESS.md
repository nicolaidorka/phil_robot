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

## Rollback
- Legacy `.ino` found → flash it, set `backend="legacy"`, restore the 4 backup .json. Byte-identical.
- Not found → stay on v2-from-repo + recalibrate (always available). A Teensy can't be bricked
  (PROGRAM-button mass-erase always re-flashes).
