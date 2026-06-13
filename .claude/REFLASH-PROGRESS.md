# Phil microstep reflash — PROGRESS & hardware facts (travels with the repo)

Active execution log for the microstep reflash (companion to the background/plan
in [FUTURE-microstep-reflash.md](FUTURE-microstep-reflash.md)). Everything here is
committed so it survives sessions and machines. **Decision: GO** (2026-06-12).

> **FLASHED + WORKING (2026-06-13).** Motors jog, frame persists across reconnects &
> power-cycles. The hard-won firmware/driver gotchas (INITIALIZE needed for movement,
> the SET_POSITION velocity-mode trap, frame restore) are in
> **[V2-FIRMWARE-NOTES.md](V2-FIRMWARE-NOTES.md)** — read that before touching v2.
> Now: clean-teach the 96-well boundary for precise well-hitting.

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

MUST FIX before flashing:
- [x] **`.ino` include** switched to `#include "def_phil.h"` (def_octopi_80120 commented out).
- [x] **backend="v2" wired** in robot.py: `elif backend=="v2"` builds V2Microcontroller; v2 connect path does
      ONE `initialize_drivers()` (applies def_phil.h current/ustep/ramp), no auto-`reset()`.
- [x] **Count constants rescaled** via a single `self._ustep_scale` (=32 for v2, 1 for legacy/sim) set in
      __init__: MOVE_CHUNK_USTEPS, APPROACH_PRE/CONFIRM_TOL/OK_USTEPS/MAX_CORRECTION, FRAME_RESET_THRESHOLD all
      ×scale; inline noise gate → `2*APPROACH_OK_USTEPS`; `_MAX_SPAN_USTEPS * self._ustep_scale` at use-site.
      Verified by import: v2 → CHUNK 1280, TOL/OK 256, MAXCORR 960, FRAME 2560; legacy/sim unchanged.
- [x] **jog_teach** takes `--v2` (builds backend=v2) and scales STEPS by `bot._ustep_scale` for re-teaching.
- [x] **Toolchain + COMPILE DONE (2026-06-12).** Installed arduino-cli 1.5.1 (~/bin), teensy:avr@1.61.0 core,
      FastLED + PacketSerial libs. **def_phil.h COMPILES CLEAN for Teensy 4.1** (FLASH code 38640 B, tons free).
      Build cmd: `PATH=~/bin:$PATH arduino-cli compile --fqbn teensy:avr:teensy41 --output-dir firmware/build \
      firmware/octopi_firmware_v2/main_controller_teensy41`. Artifact: firmware/build/main_controller_teensy41.ino.hex (152K).
      Compile surfaced + FIXED two more def_phil.h gaps (configs were never built standalone): added axis indices
      x/y/z and flip_limit_switch_x/y (copied from def_octopi.h, same V4 board).
- [ ] **FLASH prereq — udev rule** (for non-root Teensy access): `sudo cp` PJRC 00-teensy.rules to
      /etc/udev/rules.d/ (compile warned it's missing). Only needed to flash, not to build.

AXIS MAPPING CAVEAT (bring-up): def_phil.h uses def_octopi's x=1, y=0, z=2 (X and Y SWAPPED per the board
layout). This decides which motor a MOVE_X/Y/Z drives. FIRST bring-up step: jog X a few steps, confirm the
intended arm moves; if wrong, swap x/y in def_phil.h and recompile. (Re-teach happens on v2 regardless, so
either mapping ends up self-consistent — but get it right so X/Y labels match the physical arms.)
- [ ] **mm-convenience path** (constants.MICROSTEPPING_*=8) is NOT yet 256: read_position()/move_to(mm)/soft
      limits read 32x off on v2. NOT in the goto/teach critical path (those are count-based via the teach table),
      so deferred; set MICROSTEPPING_*=256 if/when the mm helpers are used on v2. (OPEN, low priority)

ROLLBACK FIRMWARE — can we get the current custom firmware back? (searched 2026-06-12)
- EXHAUSTIVELY confirmed NOT on this machine (2026-06-12): swept whole FS for octopi firmware
  (TMC4361/MOVETO) — only the restored v2 exists; no 6-byte/20-byte source or .hex; git dangling/stash
  clean; other .ino on disk are unrelated (stepper test sketches, a separate Fluidics Teensy device).
- NEVER committed to this repo (git history only ever had octopi_firmware_v2 = the 8-byte protocol).
- A Teensy 4.x flash CANNOT be read back → the running firmware is unrecoverable from the chip.
- Upstream octopi-research is 8-byte (= our v2); the 6-byte custom build matches no upstream release.
- BEST leads for a true byte-identical rollback: ask **Hongquan Li** (Squid/octopi author, likely built Phil's
  firmware) / check his repos & any USB/lab machine. Until found, realistic rollback = flash v2-from-repo +
  recalibrate (always works; a Teensy can't be bricked). The legacy calibration backup only helps IF the
  legacy firmware is found.

NOTED (not blocking bring-up):
- X/Y current scale lands at CS=14 (firmware comment prefers >16); harmless, bump X/Y to ~570mA only if weak.
- v2_mc set_max_velocity_acceleration is a no-op (relies on def_phil.h ramps) → CLI set_speed/apply_motion_profile
  are inert on v2; fine for bring-up.
- THETA status bytes [14-17] are never written by firmware → theta_pos is garbage (unused; ignore).
- robot.py home_arms() polarity must be corrected before any v2 limit-switch homing (we avoid homing at bring-up).

## Second full agent review (4 agents, 2026-06-12) — bugs FIXED
- [x] **home_arms() enable/disable polarity** was inverted for v2 (firmware: 0=disable,1=enable) — it would
      energize BOTH arms while homing one. Fixed in robot.py (harmless to legacy, which no-ops the call).
- [x] **Stale phil_frame.json on v2**: a legacy reanchor offset would apply 32x-too-small. robot.py now resets
      frame_correction to identity on the v2 backend (re-anchor fresh on v2).
- [x] **mm-path mis-scaled 32x on v2** (Z safe-lift in home, read_position): _move_axis_abs multiplies by
      _ustep_scale; read_position divides by it. (goto/teach were already pure count-path, unaffected.)
- [x] **APPROACH_CONFIRM_TIMEOUT** 2.0s too short for v2's ramped legs -> 8.0s on v2 (per-leg settle).
- [x] **_SNAP_IDENTITY** anchor snap threshold scaled by _ustep_scale.
- [x] **Driver completion detection**: _resync_and_parse now scans EVERY queued packet for the COMPLETED ack
      (was only the last -> busy flag could stick until timeout, risking a move issued mid-motion).
- [x] **Driver thread-safety**: position writes + get_pos() now use the (previously dead) lock for a consistent
      (x,y,z) snapshot; theta (never written by firmware) hard-set to 0 instead of reading garbage.
- [x] **X/Y motor current 500->560 mA** so the TMC2660 current-scale code reaches the firmware-recommended >=16
      (below it = coarse regulation -> lost microsteps). Still safe-low for the NEMA-17; recompiled clean.
- [x] **.gitignore** now ignores *.bak (config backups were untracked-but-not-ignored).
Verified: firmware recompiles clean (FLASH 38640 B); `PhilRobot(backend='v2')` -> scale 32, CONFIRM_TIMEOUT 8.0,
identity frame; sim/legacy unchanged. Driver+jog_teach import clean.
KNOWN-MINOR (not blocking, documented): wait_till_operation_is_completed still force-clears the busy flag on a
genuine timeout (bounded fallback; COMPLETED detection is now reliable); firmware checksum-error status isn't
surfaced (relies on timeout); some "full-step" code comments are stale on v2 (cosmetic); R_sense 0.22/0.43 and
the x=1/y=0 axis map still need a one-time bring-up confirmation.

## Third review (4 agents, 2026-06-12) — found NEW hazards prior passes missed; FIXED
Agents 1-2 re-verified the second-review fixes are correct w/ no regression. Agent 3 (holistic) found:
- [x] **CRITICAL: stale legacy-scale teach data live on v2.** config/phil_teach.json is byte-identical to the
      legacy backup; on v2 goto would replay legacy counts as raw microsteps (32x off). FIX: TeachTable now has a
      `ustep_scale` marker; on v2 with unmarked/legacy data the robot DROPS teach+kin_model+well_map and prints a
      loud "re-teach: jog_teach --v2 --all" (goto disabled until then). jog_teach --v2 stamps ustep_scale=256.
- [x] **CRITICAL: home()/home_arms()/home_z() do REAL limit-switch homing on v2** (legacy no-op'd it) into
      unverified switches. FIX: _block_homing_on_v2() raises on the v2 backend; use set_home()/reanchor to zero.
- [x] **384-well corners hardcoded to 96** (A1/A12/H1/H12) in teach interpolation + ANCHOR_WELLS. FIX: added
      plate_corners(plate); interpolation + self.ANCHOR_WELLS now derive from the plate (384 -> A1/A24/P1/P24, verified).
Verified: v2 construct clears stale teach + blocks homing; legacy keeps 72 wells + homing; 384 corners correct;
all files compile; legacy resolve unaffected.
KNOWN/DOCUMENTED (not fixed, bring-up items): W1 does opening serial reset the Teensy counter? (Teensy 4.x
usually does NOT auto-reset on DTR, unlike Arduino — VERIFY on hardware; affects "frame preserved" claim).
W4 the joint-space goto path bypasses mm soft-limits (firmware MOVETO is unclamped) — the stale-data guard
removes the main hazard; consider a coarse joint-count sanity bound later. N: some legacy-era code comments
("firmware ignores accel", "8-ustep grid") are stale on v2 (cosmetic).

## Final review (4 agents, 2026-06-12) — verdict GO; bring-up bugs FIXED
Go/no-go audit: firmware compiles, all backends construct correctly, every claimed fix present -> GO.
Bring-up walkthrough found workflow bugs (now fixed):
- [x] **set_home() didn't zero on v2** (firmware RESET only clears cmd_id, doesn't zero the counter) yet printed
      success. FIX: set_home() on v2 uses zero_x/y/z (HOME_OR_ZERO_ZERO); legacy keeps reset()+init.
- [x] **CLI couldn't select v2** (`--backend` choices lacked it). FIX: added "v2".
- [x] **jog_teach couldn't target a 384 / separate teach file.** FIX: added `--labware <name>` and `--teach
      <path>` so 96 and 384 teach into separate JSONs. (e.g. `jog_teach --v2 --all --labware <384.json> --teach config/phil_teach_384.json`)
- [x] **goto/gotopos silently skip the Z lift if travel-Z unset** (collision risk, esp. for WASTE). FIX: both
      now warn loudly when z_travel_usteps is None. (Set `travelz <usteps>` before any goto.)
Confirmed NOT a bug (agent misanalysis): the mm-path *32 scale is correct for Z (the real-mm axis: ratio IS 32);
only the notional X/Y "mm" differ, and those aren't physically meaningful for the rotary arm.
Verified: compiles; CLI accepts v2; 384 labware loads (16x24, corners A1/A24/P1/P24); all construct clean.

READINESS: code is GO. Human steps before/at flash: sudo install udev rule; flash firmware/build/*.hex;
first-jog verify axis map (x=1/y=0) + direction + motor temp; on v2 zero with set_home (NOT homing);
re-teach `jog_teach --v2 --all`; set `travelz`; then goto/teach WASTE. Verify Teensy doesn't reset-on-connect.

## Rollback
- Legacy `.ino` found → flash it, set `backend="legacy"`, restore the 4 backup .json. Byte-identical.
- Not found → stay on v2-from-repo + recalibrate (always available). A Teensy can't be bricked
  (PROGRAM-button mass-erase always re-flashes).
