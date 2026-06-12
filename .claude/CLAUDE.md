# CLAUDE.md — Phil robot

Guidance for working in this repo. Detailed docs alongside this file in `.claude/`:
[ARCHITECTURE](ARCHITECTURE.md) · [FINDINGS](FINDINGS.md) · [RULES](RULES.md).

## What Phil is

Phil is an **articulated 5-bar arm robot** that holds one outlet/nozzle over a
96-well plate. **It is NOT a microscope** — it reuses the Squid/octopi
microscope codebase (`software/control/`) only for the Teensy motor firmware.

- **X and Y are rotary arm joints** (a 5-bar parallel linkage: two base motors,
  each driving an arm; the two arms meet at the outlet). They are *not* a
  Cartesian stage. **Z** is vertical (up/down).
- Open-loop steppers, **no encoders**, some **backlash**. Hardware precision
  floor ≈ 1–2 mm.

## How to connect

Run from the **`software/`** directory (the Squid config is found there):

```bash
cd software
python3 -m phil.cli                 # interactive control (real hardware)
python3 -m phil.cli --simulate      # no hardware
python3 -m phil.selftest --move     # connection + feedback + tiny jog test
```

Or in Python:
```python
from phil import PhilRobot
bot = PhilRobot(backend="legacy")   # legacy = this Teensy's older firmware
bot.connect()
bot.goto_well("B10")
bot.close()
```

The controller is a Teensy on `/dev/ttyACM1` (auto-detected by manufacturer
"Teensyduino"). There's also an Opentrons Flex on `/dev/ttyACM0` — unrelated.

## What works

- **`goto <any well>`** — joints computed from the fitted **5-bar kinematics**
  (`phil/geometry/kinematics.py`), no per-well teaching. Verified on hardware (A1, E6,
  B10, G9). `goto` priority: kinematics → RBF map → exact taught → affine.
- **Any labware from its JSON** — `--labware "<name>"`; wells come from the
  plate's JSON mm through the same geometry. Assumes the plate sits in the same
  physical spot. Default plate: Eppendorf twin.tec LoBind 96 PCR.
- **X/Y/Z jog**, teach console (`phil/jog_teach.py`, arrow keys + auto-approach),
  self-test, simulation backend.
- **Startup/shutdown check habit**: **A1 is the anchor well.** The CLI verifies
  on A1 at startup (`check`) and parks on A1 at shutdown, so you always confirm
  the frame. `check [well]` runs it anytime; `--no-check` skips.
- **Power-cycle / bump recovery**: a crash, power loss, or a hard accidental
  push does **not** require re-teaching. The geometry is permanent; only the
  joint *counter* shifts (a constant offset). On connect the robot **auto-detects**
  a likely reset (live joints vs last saved pose) and warns. To recover: jog the
  outlet onto **A1** and run `reanchor` (defaults to A1). **No re-teaching, ever.**
  Note: a bump that skips steps mid-session can't be auto-detected (no encoder) —
  run `check` if you suspect one.
- **Sharper edge accuracy (`anchor`)**: `reanchor` corrects a pure translation. For a
  better fix when far-edge wells sit ~1 mm off, center each of the **4 corners**
  (A1, A12, H1, H12) and `anchor <corner>` each, then `anchor fit` — it fits a small
  **affine** joint-frame correction (offset + scale + rotation) over the 5-bar model.
  Convex + clamped + identity-until-fit, so it can't harm the calibration; the model
  and teach table are never touched. Won't beat the ~1 mm backlash floor.

## Critical rules (see [RULES](.claude/RULES.md))

1. **Never hand-move the arms to "set" a position** — open-loop, not tracked,
   and forcing the motors loses steps. Position only changes via *commanded* jogs.
2. **Keep the Teensy powered** to preserve the joint frame; after a power-cycle
   use `reanchor`, don't re-teach.
3. **Jog small** — rotary joints, very sensitive (~50 usteps per well; a few
   hundred usteps crosses the whole plate / runs off the edge).
4. The legacy connect does **not** reset/zero on connect (preserves the frame).
   `sethome`/`h` zeros deliberately; only do that during a fresh initial teach.
5. Don't blind-fire HOME — limit-switch homing is unverified on this firmware.

## Key files

```
software/phil/
  robot.py              PhilRobot: connect, jog, goto_well, teach, reanchor  <- core
  paths.py              single source of truth for config/labware locations
  constants.py          stepper geometry + motion defaults
  cli.py                interactive shell
  jog_teach.py          arrow-key teach console (auto-approach)
  selftest.py           hardware self-test
  geometry/             mm <-> joint models
    well_plate.py         loads labware JSON (by name from labware/)
    teach.py              teach table (per-well joint positions)
    calibration.py        affine fallback
    kinematics.py         5-bar geometry fit + inverse kinematics  <- the solution
    well_map.py           RBF curve-fit fallback (needs scipy)
  hardware/
    legacy_mc.py          driver for this Teensy's 6-byte/20-byte firmware
  labware/              all plate JSON (default: eppendorf_twintec_lobind_96_pcr)
  config/               phil_kinematics.json, phil_teach.json, phil_calibration.json,
                    phil_frame.json (reanchor offset + power-cycle detection)
```

## Re-calibrating from scratch (only if geometry changes)

1. `python3 -m phil.jog_teach` — teach ~10 spread wells (4 corners + 4 edge
   midpoints + 2 middle). On the first well, center it and press `h` (home).
2. `fitkin` in the CLI (or it's auto-saved) — fits the 5-bar to ~0.2 mm RMS.
3. Done — `goto` any well, any plate.
