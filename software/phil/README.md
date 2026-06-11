# Phil arm control (`phil`)

Calibrated, smooth, configurable well navigation for the **Phil** robot.

Phil is an **articulated 5-bar arm**: two motors drive two arms that meet to
hold a single outlet over the well plate, plus a Z axis that raises/lowers it.
The X and Y motors are **rotary joints (~180° range each)** — *not* a Cartesian
stage — so a well is reached by setting the two joints (and Z) to the right
positions. Because the joints are rotary, step counts map to angles: small
counts swing the arm a lot. Move gently.

It is driven over USB through the Teensy controller. **The flashed firmware
uses an older protocol than this repo's `control/microcontroller.py`** (6-byte
commands / 20-byte status packets vs 8/24), so this package talks to it with a
dedicated `legacy_mc.py` driver — no reflashing required.

## How "go to a well" works: fitted 5-bar kinematics

The two motors drive a **5-bar parallel linkage**, so the outlet position is a
closed-form function of the two joint angles. You teach ~10 wells spread across
the plate (jog the outlet over each, once), and `phil/kinematics.py` **fits the
arm's real geometry** (pivot positions, link lengths, ustep↔angle scale) from
those points — typically to ~0.2 mm. After that, **every** well on **every**
plate is computed by inverse kinematics from its JSON mm coordinates — no
per-well teaching, no interpolation sag in sparse regions.

`goto` resolution order: **5-bar kinematics** → RBF curve-fit → exact taught →
affine. Switching labware is just loading another JSON (`--labware`), as long as
the plate sits in the same physical spot.

> Earlier/fallback maps are kept: an RBF curve-fit (`well_map.py`) and an affine
> (`calibration.py`). The kinematic model supersedes them when fitted.

### Teaching (one-time, ~10 wells)

Use the arrow-key console; once a few wells are in, it **auto-approaches** each
next target so you only nudge the last bit:

```
python -m phil.jog_teach              # guided: 4 corners + 2 middle, then refine
python -m phil.jog_teach A6 H6 D1 D12 # add specific wells (auto-approached)
```
Then refit: in the CLI, `fitkin` (or it's saved to `config/phil_kinematics.json`).

## Quick start

Run from the **`software/`** directory:

```bash
cd software
python -m phil.cli                 # real hardware (legacy backend)
python -m phil.cli --simulate      # no hardware; exercise the workflow
python -m phil.selftest --move     # connection + feedback + tiny jog test
```

In the shell:

```
phil> joints                 # show arm joint positions (usteps)
phil> jx 200                 # jog the X arm +200 usteps (small!); jy / jz too
phil> teach A1               # save current joints as well A1
phil> teach A12
phil> teach H1
phil> teach H12              # 4 corners -> every other well is interpolated
phil> goto D6                # move the arm to well D6
phil> save                   # persist the teach table to config/phil_teach.json
```

Teach more wells any time (`teach E7`) to refine accuracy where it matters.

## Files

| File | Purpose |
|------|---------|
| `legacy_mc.py` | `LegacyMicrocontroller` — speaks this Phil's 6-byte/20-byte firmware (threaded reader, unit conversion to repo usteps). |
| `phil_robot.py` | `PhilRobot` — connect/jog/teach/`goto_well` with coordinated arm motion; selectable backend (`legacy`/`stock`/`sim`). |
| `teach.py` | `TeachTable` — per-well joint positions + corner interpolation, save/load. |
| `well_plate.py` | `WellPlate` — loads the labware JSON, well↔(row,col) mapping. |
| `labware/corning_96_wellplate_360ul_flat.json` | 96-well plate geometry (used for the well grid / interpolation). |
| `calibration.py` | Affine Cartesian calibration — kept for a true XY-stage Phil; not used by the articulated arm. |
| `constants.py` | mm↔ustep math and motion constants. |
| `cli.py` / `selftest.py` | Interactive teach shell / hardware self-test. |

## Firmware protocol (this Teensy)

Reverse-engineered and verified over USB:

- command frame = **6 bytes**: `[cmd_id, opcode, p0, p1, p2, crc8]`
- status packet = **20 bytes**: `[cmd_id, status, X(4 BE), Y(4 BE), Z(4 BE), theta(4 BE), buttons, pad]`
- CRC-8/CCITT (poly 0x07); opcodes match `control._def.CMD_SET` (MOVE_X=0 verified)
- 256 microstepping: **commands are in full-steps, position is reported in
  microsteps** (cmd × 256 = position delta). `legacy_mc.py` converts both ways so
  the rest of the package speaks consistent "repo usteps" (1600/mm on X,Y).

## Homing / alignment

The two arm motors are aligned by homing each to its limit switch **with the
other motor disabled** (so the parallel linkage doesn't fight itself), then
zeroing — mirroring `test_20240823.py`. This is `PhilRobot.home()` /
`home_arms()`. It is **opt-in** (CLI: `home yes`) because it sweeps the full
travel; make sure the workspace is clear first.
```
