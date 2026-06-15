# Units & calibration — Phil

The single most confusing part of Phil is **which "step" a number is in** and
**which calibration tool to reach for**. This is the quick reference. Mechanism
and model details: [ARCHITECTURE](ARCHITECTURE.md).

## Units glossary

| term | what it is |
|------|-----------|
| **full-step** | one motor full-step (200/rev). The legacy firmware is **commanded** in full-steps. |
| **firmware microstep** | the firmware's internal count. Legacy reports at **256 microstepping**; v2 firmware is finer still. |
| **repo ustep** | the package's internal unit (**microstepping 8**) — what `joints`/`where`/teach JSON show. `legacy_mc.py` converts at its boundary: command full-steps = repo_usteps / 8; reported repo_usteps = firmware_microsteps / 32. |
| **plate-local mm** | physical mm in the plate frame, from labware JSON. The kinematic model maps mm ↔ joints. |

**Why a "ustep" is not a fixed mm:** the arms are **rotary**, so the mm-per-ustep
at the tip changes across the workspace. Locally near the plate it's **~6 repo
usteps/mm**, **1 full-step ≈ 1 mm** of tip travel, **~50 usteps per 9 mm well** —
but these are local. **Trust the kinematic model for mm, never a single scale.**

**legacy vs v2 scale (the gotcha):** the v2 microstep firmware reports/commands
~**175 counts/mm** at the tip vs legacy ~**5.5–6** (the same ~6 repo usteps/mm
above, ×32). PhilRobot rescales every
count-based tunable by `_ustep_scale` (**32 for v2, 1 for legacy**) so the motion
logic is backend-agnostic. A saved frame records the scale it was written in
(**256 = v2, 8 = legacy**) so it's never mis-read across firmware. Bottom line:
**don't compare raw counts between backends** — they're in different units.

## Which calibration tool, when

All four leave the **kinematic model untouched** unless noted; the production
model is permanent (`phil_kinematics.json`). Pick by *what's wrong*:

| situation | tool | what it does |
|-----------|------|--------------|
| Power-cycle / counter reset / tip uniformly off | **`rehome --v2`** | jog to **true A1 centre** → ENTER zeros there → restores the entire taught frame. The blessed recovery. No re-teach. |
| Same as above, 1-well translation only | **`reanchor [well]`** | jog onto A1 (default), recompute a constant frame offset (`phil_frame.json`). Translation only. |
| Far-edge wells ~1 mm off, *systematic* | **`anchor` ×4 → `anchor fit`** | center A1/A12/H1/H12, capture each, fit a small **affine** (offset+scale+rotation) over the model. Convex, clamped, identity-until-fit — can't harm calibration. |
| "It's a bit off" — need to know *why* | **`measure --v2`** | diagnostic (below). |
| Suspect a *mis-taught* well (typo'd jog, bad approach) | **`stepcheck`** | checks each taught well against its grid-neighbours' joint step-delta; flags the outliers (`--tol-mm`). Run before chasing a model bug. |
| A specific well is genuinely wrong | **`jog_teach`** (teach that well), or **`tiptrack`** | record exact joints; `goto` then replays them. `tiptrack` makes the jog *observable* (live tip marker) while you center. |

Never `fitkin` (refit the 5-bar) for a small error — it's non-convex and can
regress. Refit only if the **arm geometry physically changed** (see RULES).

## `measure.py` — turn "a bit off" into numbers

`python3 -m phil.measure --v2 [wells...]` (run in your own terminal — needs live
keys). It drives each well **via the model**, you jog the nozzle to the **true
centre**, and it records the offset (dX, dY, mm, direction).

It exists to **classify the error**:
- **Systematic** (same offset/direction on every well) → one frame correction
  (`reanchor` / a single shift) fixes nearly all of it.
- **Per-well / random** → genuine model or teaching error → teach those spots.

- Default = a diagnostic spread of mostly-untaught interior + edge wells, plus a
  couple of taught refs (A1/D6) as the backlash baseline. Or pass specific wells.
- Keys: arrows jog X/Y, `a`/`z` Z, `+`/`-` step size, **Enter** record, `n` skip,
  `q` finish.
- Saves offsets to `config/phil_offsets.json`. `--apply` *also* re-teaches each
  centred well (exact).
- Acceptance is **by eye** (open-loop) — you judge "on the centre," then Enter.

> **`tiptrack` caveat:** its live marker is the **model's** converged tip
> (`KinematicModel.forward`), not an independent readout of the *physical* nozzle.
> If the frame is offset, the marker is offset with it. So `tiptrack` makes
> jogging *observable* and is great for teaching — but final physical acceptance
> is still **your eye on the real nozzle**, same as everywhere else.
