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

## How "go to a well" works: boundary taught, interior interpolated

The two motors drive a **5-bar parallel linkage**, so the outlet position is a
closed-form function of the two joint angles, and `geometry/kinematics.py` can
**fit the arm's real geometry** (pivots, link lengths, ustep↔angle scale) from
taught wells. But fit from only ~10 spread wells, the model **overfits** — it
has to extrapolate to the plate edges, where leave-one-out error reaches
~1.5–4.3 mm.

The fix is **teach the boundary, refit, interpolate the interior**: teach the
perimeter rows (**A–E + H**, 72/96 — jog the outlet over each, once), refit the
5-bar on all of them (RMS ≈ 0.42 mm), and the model then only has to
**interpolate** the few interior wells (rows **F/G**) that the taught boundary
brackets — verified ~0.5 mm at F6 (column 1 is the weak edge).

`goto` resolution order: **exact taught → 5-bar kinematics → RBF curve-fit →
affine**. A taught well always replays its recorded joints; untaught wells (the
interior, or any other labware) come from the refit model. Switching labware is
just loading another JSON (`--labware`), as long as the plate sits in the same
physical spot — the model maps its mm.

> Fallbacks below the model: an RBF curve-fit (`well_map.py`) and an affine
> (`calibration.py`).

### Teaching

Use the arrow-key console; it **auto-approaches** each next target (taught spot
if known, else the model) so you only nudge the last bit:

```
python -m phil.jog_teach --all        # walk the boundary rows in snake order (resumable)
python -m phil.jog_teach A6 H6 D1 D12 # add/refine specific wells (auto-approached)
```
In `--all`, **do not press `h`** (it zeros the frame and wrecks taught wells);
`n` skips a well (e.g. the interior F/G left to the model), `s` saves, `q` quits.
Then refit: in the CLI, `fitkin` (saved to `config/phil_kinematics.json`).

## Quick start

Install and entry points are in the [project README](../../README.md). Run from the
**`software/`** directory (e.g. `python -m phil.cli`, `--simulate` for no hardware).

In the shell:

```
phil> joints                 # show arm joint positions (usteps)
phil> jx 200                 # jog the X arm +200 usteps (small!); jy / jz too
phil> teach A1               # save current joints as well A1
phil> teach A12
phil> teach H1
phil> teach H12              # corners; teach the full boundary, then fitkin
phil> goto D6                # move the arm to well D6 (taught -> exact, else model)
phil> save                   # persist the teach table to config/phil_teach.json
```

Teach more wells any time (`teach E7`) to refine accuracy where it matters; run
`fitkin` afterward to fold them into the interior interpolation. (Four corners
alone are *not* enough — the arm is too curved for a corner interpolation to
hold; teach the boundary rows.)

## Files

| File | Purpose |
|------|---------|
| `robot.py` | `PhilRobot` — connect/jog/teach/`goto_well` with coordinated arm motion; selectable backend (`legacy`/`stock`/`sim`). |
| `paths.py` | Single source of truth for `config/` and `labware/` locations. |
| `constants.py` | mm↔ustep math and motion constants. |
| `cli.py` | Interactive teach/control shell (`python -m phil.cli`). |
| `jog_teach.py` | Arrow-key jog + teach console, auto-approach (`python -m phil.jog_teach`). |
| `selftest.py` | Hardware self-test (`python -m phil.selftest`). |
| `geometry/well_plate.py` | `WellPlate` — loads the labware JSON, well↔(row,col) mapping. |
| `geometry/teach.py` | `TeachTable` — per-well joint positions + corner interpolation, save/load. |
| `geometry/calibration.py` | Affine calibration fallback. |
| `geometry/kinematics.py` | `KinematicModel` — 5-bar geometry fit + inverse kinematics (interpolates the untaught interior; taught wells win). |
| `geometry/well_map.py` | RBF curve-fit fallback (needs scipy). |
| `hardware/legacy_mc.py` | `LegacyMicrocontroller` — speaks this Phil's 6-byte/20-byte firmware (threaded reader, unit conversion to repo usteps). |

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

> **Caution — unverified on this firmware.** Limit-switch homing has *not* been
> confirmed on this Teensy's legacy firmware and could drive an arm into a hard
> stop. Day-to-day you don't need it: the frame persists across reconnects, and
> after a power-cycle you `reanchor` rather than home (see `.claude/RULES.md`).
> Use manual `sethome` (zeros at the current pose, no motion) for a fresh teach.
```
