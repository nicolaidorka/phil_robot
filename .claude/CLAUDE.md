# CLAUDE.md — Phil robot

> ## ‼️ READ FIRST, EVERY SESSION
> Before changing anything, **read [RULES](RULES.md) and [LEARNINGS](LEARNINGS.md)** —
> they hold hard operating rules and every past failure/complaint so you don't repeat
> them. **MANDATORY going forward:** log EVERY failure mode AND EVERY piece of negative
> feedback from the user into [LEARNINGS](LEARNINGS.md) (newest first: what happened,
> root cause, the rule it burns in) before moving on. Two recurring ones already burned
> in: **never clamp/alter manual jogging**, and **never bring up Z** unless the user does.

Guidance for working in this repo. **Documentation map** (all in `.claude/`):

| doc | what's in it |
|-----|--------------|
| [ARCHITECTURE](ARCHITECTURE.md) | code map of `software/phil/` — layers, entry points, well-resolution order, kinematic model, drivers (legacy + v2), units, config files |
| [RULES](RULES.md)               | hard operating rules (never hand-move; don't blind-home; jog small; preserve the frame) |
| [TROUBLESHOOTING](TROUBLESHOOTING.md) | symptom → cause → fix for the failure modes that actually happen on this unit |
| [UNITS-AND-CALIBRATION](UNITS-AND-CALIBRATION.md) | what each "step" unit means (legacy vs v2 scale) + which calibration tool when (rehome/reanchor/anchor/measure) |
| [FINDINGS](FINDINGS.md)         | what was learned the hard way (overfit, edge error, backlash floor, protocol reverse-eng) |
| [V2-FIRMWARE-NOTES](V2-FIRMWARE-NOTES.md) · [V2-GOTO-FIXES](V2-GOTO-FIXES.md) | the v2 microstep firmware: protocol notes + the goto fixes |
| [REFLASH-PROGRESS](REFLASH-PROGRESS.md) · [FUTURE-microstep-reflash](FUTURE-microstep-reflash.md) | the microstep reflash effort: live progress + the parked plan |

The **code architecture lives in [ARCHITECTURE](ARCHITECTURE.md)** — read it first
to find your way around `software/phil/`; the "Key files" section at the bottom of
this file is the quick index.

## What Phil is

Phil is an **articulated 5-bar arm robot** that holds one outlet/nozzle over a
96-well plate. **It is NOT a microscope** — it reuses the Squid/octopi
microscope codebase (`software/control/`) only for the Teensy motor firmware.

- **X and Y are rotary arm joints** (a 5-bar parallel linkage: two base motors,
  each driving an arm; the two arms meet at the outlet). They are *not* a
  Cartesian stage. **Z** is vertical (up/down).
- Open-loop steppers, **no encoders**, some **backlash**. Hardware precision
  floor ≈ 1–2 mm.

## Goal

> ## ⭐ PRIMARY SIMPLE GOAL — get this right first
> **Teach a well once, and `goto` that well puts the nozzle back on it.** That's
> the whole job, reduced to its simplest form. If teach→goto doesn't return to the
> SAME physical spot, nothing else matters — fix this before any grid/model/coverage
> work.
>
> **Root cause (found + FIXED 2026-06-13):** `goto` always finishes its approach
> from **+X,+Y** (`_approach_joints`, pre-position −X,−Y → close in +X,+Y) to put
> backlash in a known state. The teach console used to record the raw count however
> you finished nudging (only *warning* on a Down/Left finish), so a well finished
> **−X,−Y** replayed **~one backlash gap (~2 mm) further +X,+Y** — count perfect,
> physical spot off by the slop, because teach and goto took up backlash in OPPOSITE
> directions. Confirmed on hardware (H12).
>
> **Fix (implemented):** the teach console now RECORDS the finish direction per axis
> (`jog_teach` `last_dir` → `teach_well(finish=…)` → `TeachTable` stores `"finish"`),
> and **`goto` REPLAYS the same engagement** (`goto_well` reads `finish_for_well()`
> and passes `approach=` to `_approach_joints`). So the operator centres a well in
> ANY direction and goto reproduces it — no Up/Right discipline, no tip motion at
> record time, no change to jogging (the `2026-06-11` "seated jogging" attempt that
> clamped jogs is NOT this; never reintroduce it). Wells taught before this default
> to +X,+Y (old behaviour) — re-teach to capture their real finish. Residual floor =
> the irreducible ~1 mm backlash; going below it needs a position sensor (encoders).

**Position the nozzle over the X/Y centre of any well of a 96-well plate.** Z is
only raise/lower; the target that matters is always the **X/Y centre** of the
well.

> **USE THE LABWARE JSON'S METRIC GRID.** The 96-well plate JSON states the true
> **metric (mm) positions/distances between wells** — a uniform ~9 mm grid (and it
> SHOULD always state them; if a labware file lacks them, add them). That is ground
> truth we must exploit with the stepper counts: derive **usteps↔mm locally from
> neighbour wells**, use the known equal spacing to detect mis-taught wells and to
> predict/interpolate untaught ones, and to sanity-check goto. Find a way to tie the
> JSON metric distances to the motor counts rather than treating taught counts as
> unanchored numbers. (See [LEARNINGS](LEARNINGS.md) "rigid grid, not fitkin".)

> **WORKING RULE — accuracy/calibration is about X/Y placement over wells, period.**
> Do NOT raise, discuss, or "handle" Z unless the user *explicitly* brings up Z.
> All deviation/repeatability/accuracy specs (e.g. "≤1.3 mm per well") refer to
> X/Y only. When in doubt, ignore Z. Next to the plate sits a **waste container** the arm must also reach
(a named off-plate position). The working cycle is: `goto <well>` → *(liquid
handling — NOT controlled by this code; Phil only **positions** the nozzle)* →
`gotopos WASTE` → repeat.

### Layout (top-down, as the hardware actually sits)

```
   X motor (left arm)                         Y motor (right arm)
        ⟳  rotary shoulder            rotary shoulder  ⟳
          \  proximal              proximal  /
           \  distal ───── nozzle ───── distal  /
                              │ (X/Y centre = target; Z = up/down)
        ┌──────────┐        H  G  F  E  D  C  B  A   ← rows A→H (right→left)
        │  WASTE   │      ┌────────────────────────┐
        │ container│   1  │ ·  ·  ·  ·  ·  ·  ·  ⊕ │ ⊕ A1 = UPPER-RIGHT = (0,0)
        │ (beside  │  …   │ ·  ·  ·  ·  ·  ·  ·  · │   cols 1→12 run top→bottom
        │  plate)  │  12  │ ·  ·  ·  ·  ·  ·  ·  · │   down the RIGHT edge
        └──────────┘      └────────────────────────┘
                            H1=top-left   A12=bottom-right  H12=bottom-left
```

**Plate orientation (known, fixed):** rotated 90° with **A1 in the UPPER-RIGHT**
corner — A12 = lower-right, H1 = upper-left, H12 = lower-left. Columns 1→12 run
top→bottom down the right edge; rows A→H run right→left across the top
(12 wells tall × 8 wide in the robot frame). A naive A1-origin + 9 mm-spacing
formula comes out **transposed** unless this rotation is accounted for — trust
the **taught** joints, not the nominal grid (the model-vs-nominal corner error is
~11 mm precisely because the nominal grid ignores this rotation).

**Arm orientation (known):** 5-bar linkage, **X = left-arm rotary joint, Y =
right-arm rotary joint**, the two arms meeting at the single nozzle; **Z** =
vertical. A1's true centre is the frame zero `(0,0,0)`; every taught well is
relative to it. Re-zero with `rehome --v2` onto the **true centre of A1** — never
re-teach.

**Reaching the X/Y centre — the open-loop caveat.** "0 drift / frame intact" on
connect only means the *counter* agrees with itself; it does **not** prove the
nozzle is physically over the well centre (no encoder). A `goto` cannot "centre"
a frame that is physically offset — if the tip is visibly off-centre, the fix is
`rehome --v2` (recentre on true A1), not another goto. Confirm centring **by
eye**, then trust the taught frame.

### What "over the centre" means (acceptance)

A well is "hit" when the nozzle is over its **X/Y centre** within the hardware
floor (~1–2 mm; backlash-limited — see [FINDINGS](FINDINGS.md)). Because the arm
is open-loop, **counter == target is necessary but not sufficient** — final
acceptance is **visual, by eye**, not the joint readout. A taught well replays
its exact joints, so a taught well that looked centred when taught is your most
reliable target; untaught interior wells (F/G) lean on the model.

### Achieving the goal — operational recipe

Order matters: **set the lift first**, then move. Run from `software/`.

1. **Set a safe travel-Z (do this once per session, before any cross-plate
   goto).** `travelz` with no argument captures the *current* Z as the safe
   height, so jog Z up to clear the tallest plate wall, then capture it — no
   ustep math, firmware-agnostic:
   ```
   python3 -m phil.cli
   phil> jz 400            # raise nozzle (repeat until it clears the plate wall)
   phil> travelz           # capture current Z as the safe travel height
   ```
   (+Z is UP / safe; smaller Z is DOWN into the well.)
2. **Go to a well centre** — coordinated X/Y, with the lift→traverse→descend the
   travel-Z now enables:
   ```
   phil> goto B10
   ```
3. **Confirm centring by eye** (the open-loop acceptance). If a well is visibly
   off and it's a *frame* shift (e.g. after a bump/power-cycle), recover with
   `rehome --v2` onto true A1 — **never re-teach**.
4. **Teach the WASTE container** (needed for the dispense cycle; not yet taught):
   jog the nozzle over the waste opening (Z high enough to clear the plate wall
   on the way), then save it as a named off-plate spot:
   ```
   phil> teachpos WASTE
   phil> gotopos WASTE     # lift -> traverse -> descend to it; verify by eye
   ```
5. **Run the cycle:** `goto <well>` → *(liquid handling, done outside this code)*
   → `gotopos WASTE` → repeat. Validate a batch of wells open-loop with
   `sweep <w1> <w2> ...`
   (predicted-vs-reached error; taught wells ≈ 0, interior wells test the model).

### Status / TODO (as of this writing)

- ✅ Frame **stable** after the connect-time snap fix (2026-06-15); 8 wells taught in
  one consistent frame: **4 corners A1/A12/H1/H12 + A2/A3/D6/E7**.
- ✅ **Untaught wells now covered by the LOCAL grid predictor** (`predict_grid`,
  distance-weighted) — interior leave-one-out **~1.7 mm** (at the hardware floor),
  every untaught well interpolates inside the taught hull (4 corners taught), none
  clamps off-plate. **You do NOT need to teach all 96.** Only two sparse clusters
  (left-mid ~F4/G4, right-mid ~C9/D10) lack a nearby anchor; teach ONE well in each
  if a hardware check shows them off. (The old "24/96 L-shape, re-teach the 72-well
  boundary" plan is superseded — see [LEARNINGS](LEARNINGS.md) 2026-06-15.)
- ⚠️ **`travelz` is unset** (`z_travel_usteps: null`) — `goto`/`gotopos` move
  WITHOUT a lift and can drag the nozzle. **Set it first** (step 1 above).
- ⚠️ **WASTE not taught yet** (`named: {}`) — needed for the cycle (step 4 above).

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
bot = PhilRobot()                   # defaults to constants.DEFAULT_BACKEND ("v2",
                                    # the flashed firmware). Pass backend="legacy"
                                    # ONLY if the board is rolled back to old firmware.
bot.connect()
bot.goto_well("B10")
bot.close()
```

The controller is a **Teensy, auto-detected by manufacturer "Teensyduino" /
serial `16640550`** — so you never pass a port. **The `/dev/ttyACMx` number is
NOT stable** (it has come up as both `ttyACM0` and `ttyACM1` depending on
enumeration order); trust the SN/mfr auto-detect, not a fixed number. An
Opentrons Flex also enumerates on a `ttyACMx` — unrelated, and the auto-detect
skips it. (Other docs that name a specific `ACMx` are just noting what it was
that day — the SN is the source of truth.)

## What works

- **`goto <well>`** — for a taught well, replays its **exact recorded joints**
  (**taught wins, always**); for an untaught well, the refit 5-bar model. `goto`
  priority: **exact taught → kinematics → RBF map → affine**.
  > ⚠️ **CURRENT COVERAGE (post-reflash, verified 2026-06-13): only 24/96 wells
  > are taught** in the v2 frame — an **L-shape** (row A + column 1 + C9/D6/E4/
  > F9/H12). **The other 72 wells are model-only**, and the model extrapolates
  > badly off the L: corner leave-one-out ≈ **12–13 mm**. So only the 24 taught
  > wells are known to the ~1–2 mm floor; the rest can be ~1 cm off.
  > The old **72-well boundary teach (rows A–E + H, RMS 0.42 mm) is PRE-REFLASH**
  > (`config/pre-reflash-backup/`) and **invalid** in v2 units — it was never
  > rebuilt. **Fix:** re-teach the boundary in v2 (below), then `fitkin`.
  Teach with `python3 -m phil.jog_teach --all --v2` (snake order, resumable —
  `s` saves, `q` quits; `n` skips interior F/G; **never `h`**).
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

## Critical rules (see [RULES](RULES.md))

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
  cli.py                interactive shell (main surface)
  jog_teach.py          arrow-key teach console (auto-approach; --all walks every well)
  drive.py              free arrow-key drive — move the arm, NO teaching
  rehome.py             the blessed recovery: jog onto true A1 centre -> set home -> confirm
  measure.py            drive each well via the model, jog to true centre, record the offset
  tiptrack.py           live GUI: marker on the converged tip follows the arm as you jog (+ teach/fitkin)
  stepcheck.py          flags mis-taught wells via joint-space neighbour step-delta consistency
  viz.py                shared matplotlib helpers for plate visualizations (no pyplot)
  selftest.py           hardware self-test
  geometry/             mm <-> joint models
    well_plate.py         loads labware JSON (by name from labware/)
    teach.py              teach table (per-well joint positions)  <- primary
    calibration.py        affine fallback
    kinematics.py         5-bar geometry fit + inverse kinematics  (fallback for untaught wells)
    well_map.py           RBF curve-fit fallback (needs scipy)
  hardware/
    legacy_mc.py          driver for this Teensy's 6-byte/20-byte firmware (backend=legacy)
    v2_mc.py              driver for the newer microstep firmware       (backend=v2)
  labware/              all plate JSON (default: eppendorf_twintec_lobind_96_pcr)
  config/               phil_kinematics.json, phil_teach.json, phil_calibration.json,
                    phil_frame.json (reanchor offset + power-cycle detection)
```

## Teaching / re-calibrating

The arm is open-loop with backlash, so the dependable approach is **teach the
wells you need and let `goto` replay them exactly** — a taught well short-circuits
the model entirely (`_resolve_raw`: the `is_taught` branch wins first), so the
model is only ever a fallback for *untaught* wells.

**Current state (2026-06-15):** stable v2 frame with a growing taught set (10+
wells incl. all 4 corners). Untaught wells fall back to the **rigid-grid predictor**
(`predict_grid`, interior LOO ~1.7 mm = at the hardware floor). The old
"24/96 L-shape" / "teach the 72-well boundary then `fitkin`" plan is **superseded** —
see the Status section above and [LEARNINGS](LEARNINGS.md).

To teach more wells (or all 96):

1. `python3 -m phil.jog_teach --all` — walks the wells in snake order,
   auto-approaching each from the last (taught spot if known, else the model).
   Nudge to center, `Enter` to record, `n` to skip, final approach in one
   direction (backlash). **Do NOT press `h`** — it zeros the frame and wrecks the
   wells already taught. `s` saves progress, `q` quits — rerun `--all` to resume
   (already-taught wells are re-approached so you can confirm or `n` past). Saves
   are auto-backed-up and a catastrophic shrink is refused (see [LEARNINGS](LEARNINGS.md)).
2. ⛔ **Do NOT run `fitkin` after a normal teach pass.** It refits the **5-bar
   kinematic model**, which is **retired to dead-last** (overfits/extrapolates) and
   is **non-convex — it can REGRESS a good calibration** (see [RULES](RULES.md)). It
   does **nothing** for taught wells (they short-circuit the model) and untaught
   wells use the rigid grid, not the 5-bar. The only "after teaching" steps that
   matter are **set `travelz`** and **teach `WASTE`**. `fitkin` is reserved for the
   rare case the arm geometry physically *changes* (below).

If the **arm geometry physically changes**, the kinematic model is stale: do a
*fresh* teach of a spread of wells — on the FIRST well only, center it and press
`h` (home) to zero the frame — then `fitkin` to refit (~0.2 mm RMS over the fit
set, but it does not generalize to the edges — see [FINDINGS](FINDINGS.md)).
