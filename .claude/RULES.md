# Rules — operating & development

## Safety (physical arm)
- **Jog small.** Rotary joints, very sensitive (~50 usteps/well). A few hundred
  usteps crosses the whole plate or runs off the edge. Start with small steps;
  the `jog_teach` console defaults to multiples of 8 usteps (≈1 mm).
- **Lift / clearance before big XY moves** if anything is on the deck. `goto`
  moves XY then sets Z; set a travel Z if you need clearance.
- **Don't blind-fire HOME / limit-switch homing.** It's unverified on this
  firmware and could drive an arm into a hard stop. Use manual `sethome` (zeros
  at the current pose, no motion) instead.
- When commanding motion you can't see, move in small bounded steps and confirm
  with the operator.

## Don'ts
- **Don't hand-move the arms to set a position.** Open-loop, not tracked, and it
  can lose steps. Position only changes via commanded jogs.
- **Don't `reset()` / `initialize_drivers()` mid-session on legacy** unless you
  intend to zero the frame — both wipe the joint counter. (Legacy connect skips
  them on purpose.)
- **Don't use `backend="stock"`** on this unit — the 8/24 protocol is rejected.
  Use `backend="legacy"`.
- **Don't re-teach after a power-cycle.** Use `reanchor <well>` (1-well, translation).
  For a sharper edge fix, anchor the 4 corners: `anchor A1/A12/H1/H12` then `anchor fit`
  (affine correction; never refit the 5-bar — that's non-convex and can regress).

## Day-to-day
- Run from `software/`. `python3 -m phil.cli`, then `goto <well>`.
- **Check on start and finish.** A1 is the anchor well. The CLI moves to A1 at
  startup so you can eyeball it, and parks on A1 at shutdown. If the outlet is
  ever NOT on A1 during the check, the frame slipped — jog onto A1 and `reanchor`.
  Run `check` anytime you suspect a bump. (`--no-check` skips the auto-moves.)
- After a **power-cycle or accidental bump**: on connect the robot warns if the
  frame looks reset. Jog the outlet onto A1 and run `reanchor` (defaults to A1).
  Calibration restored; **no re-teach** (geometry in `phil_kinematics.json` is
  permanent). A mid-session bump that skips steps isn't auto-detected (no
  encoder) — that's what the start/finish `check` is for.
- Switch plates: `--labware "<name>"` (list with `labware`); geometry maps the
  new JSON's wells.

## Teaching (the production path — taught wells beat every model)
- The 5-bar model overfits (edge LOO ~1.5–4 mm), so we **teach every well** and
  replay exact joints. `goto` is taught-first; the model only fills untaught
  wells. **72/96 taught** (rows A–E + H; rows F and G remain).
- Teach the rest: `python3 -m phil.jog_teach --all` — all 96 in snake order,
  auto-approaching each from the last. Center the FIRST well and press `h`
  (home) before Enter; then nudge + Enter per well, final approach in ONE
  direction (backlash), "over the well" is good enough. `s` saves, `q` quits —
  rerun to resume.

## Re-fitting the 5-bar (rare — only if the arm geometry physically changes)
1. Teach a spread of wells (4 corners, 4 edge midpoints, 2 middle) as above.
2. `fitkin` (or it's saved) → refits the 5-bar (in-sample RMS < ~0.5 mm; do not
   expect edge accuracy — it overfits, which is why we teach every well).
3. Re-teach the affected wells so `goto` replays measured joints, not the model.

## Development
- Keep `legacy_mc.py` matching the firmware framing (6/20, CRC-8/CCITT, 256
  microstepping). If the Teensy is ever reflashed with the repo firmware, switch
  to `backend="stock"` and the standard `constants.py` units.
- Optional deps: `scipy` (for `well_map.py` and `kinematics.py`). The package
  degrades gracefully if absent (falls back to affine).
- Verify changes in simulation first: `python3 -m phil.cli --simulate` /
  `python3 -m phil.selftest --simulate --move`.
- `constants.py` mirrors `control/_def.py`; the rotary "mm" there is nominal —
  real outlet positions come from the kinematic model, not those constants.
