# Learnings — operational mistakes to not repeat

Mistakes made while operating Phil (esp. by the assistant), logged so they don't
recur. Newest first. See also [FINDINGS](FINDINGS.md) (firmware/mechanism facts)
and [TROUBLESHOOTING](TROUBLESHOOTING.md).

## 2026-06-15 — NEAR-MISS: I told the operator to run `fitkin` after a full teach — would have regressed the calibration they'd just spent an hour building

Planning a teach-all-96 session, I parroted the stale "teach the wells → run `fitkin` afterward to fold
them into the model" recipe straight out of CLAUDE.md/RULES.md. The operator caught it: *"make sure we
don't make that mistake about fitkin again, that would have been devastating."* They're right.

- **Why it was dangerous:** `fitkin` refits the **5-bar kinematic model**, which the code has already
  **RETIRED to dead-last** (`robot.py` `_resolve_raw` doc: *"the 5-bar kinematics is retired to dead-last
  — it overfit/extrapolated badly"*) and which RULES flags as **non-convex → can REGRESS a good fit**.
  Running it right after a careful manual teach could have distorted the working calibration — undoing
  the whole session — for **zero benefit**.
- **Why zero benefit:** a taught well **short-circuits the model** (`_resolve_raw` returns the `is_taught`
  branch FIRST, before grid/curve-fit/5-bar). With all 96 taught, `goto` never consults a model at all.
  And untaught wells use the **rigid-grid predictor** (`predict_grid`), not the 5-bar — so even partial
  teach sets don't want `fitkin`.
- **Root cause (mine):** I recited the docs' teaching recipe without reconciling it against the newer
  code reality (5-bar retired, rigid grid primary, taught-wins). The docs themselves still said "run
  `fitkin` afterward" in two places — a stale instruction that contradicts the code.

**Rules this burns in:**
1. ⛔ **Do NOT run `fitkin` after a teach pass (partial OR all-96).** It's the non-convex 5-bar refit;
   it can regress a good calibration and does nothing for taught wells (they short-circuit the model).
   `fitkin` is ONLY for a genuine *physical* arm-geometry change. After teaching, the steps that matter
   are: **set `travelz`, teach `WASTE`.** **FIXED 2026-06-15:** removed the stale "run fitkin afterward"
   instruction from CLAUDE.md (Teaching section) and RULES.md and replaced both with this warning.
2. **Protect the teaching work from any save path. FIXED 2026-06-15:** `TeachTable.save()` now (a)
   snapshots the existing file to a timestamped `.backup-*` before EVERY overwrite (keeps newest 30), and
   (b) **refuses a catastrophic shrink** — if a save would lose >half the taught wells (the tiptrack
   26→6 wipe signature, or a wipe to 0) it raises `TeachShrinkGuard`, leaves the good file on disk, and
   dumps the attempted table to a `.rejected-save-*` sidecar so BOTH states survive (override with
   `save(allow_shrink=True)`; a normal 1-2 well undo is still allowed). Core `save()` previously had NO
   backup at all — only tiptrack's fit path did — so a normal/partial/crashed write could clobber the
   table irrecoverably. Verified: 20/20 tests pass + a functional test of backup + guard + override.

## 2026-06-15 — NEGATIVE FEEDBACK: I over-blamed "Z=0 dragging" for a jam that was really an EMI hang + a TWO-CLI serial conflict

The operator centred a well, taught F4/C9, everything worked — then on CLI shutdown the arm hung,
they opened a second terminal, the arm jammed, and taught F4 read off. I repeatedly attributed the
jam + frame shift to **Z=0 dragging** and pushed a travel-Z lift. The operator corrected me:
**"there was nothing wrong with the z lift."** They're right; I was wrong.

- **The ACTUAL failure chain (no Z involved):**
  1. An **EMI USB drop hung the CLI's shutdown park-on-A1 move** — it blocked forever waiting on a
     serial reply that never came (no `arrived:` line ever printed). `q` did nothing; the process
     stayed alive (confirmed `ps`: still `Sl+`).
  2. To regain control the operator **launched a SECOND CLI in another terminal**. Now **two
     processes shared `/dev/ttyACM1`** → their bytes interleave into **garbage/partial move
     commands** → the firmware drove the arm to a wrong pose and it **JAMMED** → lost steps →
     the frame shifted (taught F4 now off).
  3. The **startup twitch reappeared** for the same root cause: the Teensy **re-enumerated** under
     EMI, so `connect` saw live-counter ≠ saved and re-ran `initialize_drivers()` (the snap).
- **Why my Z blame was wrong:** Z=0 dragging is a *secondary* step-loss effect on big moves; it did
  NOT cause this jam. The jam was a **bad command from the two-process port conflict**. Harping on Z
  ignored the operator's setup and wasted their patience while the arm was jammed.
- **Recovery that worked:** killed ALL phil processes (one SIGINT left it up — needed SIGTERM/pkill),
  confirmed the 10 taught wells were safe on disk, then `drive.py` → jog onto A1 → `r` (reanchor).
  The reanchor absorbed a big translation (cx +12800 vs the prior +8528 — the jam shoved the frame
  ~26 mm). No re-teach; the teach table was never in danger.

**Rules this burns in:**
1. **NEVER run two programs on the Teensy's serial port at once.** Two opens on one `ttyACM`
   interleave into garbage moves → wrong pose → jam. **FIXED 2026-06-15:** `v2_mc` now takes an
   exclusive `fcntl.flock` on the serial fd at open → a second CLI/drive/jog_teach is **refused** with
   a clear "already in use" message (auto-released on exit, no stale lockfile; bypass with
   `PHIL_NO_PORT_LOCK=1`). And on a dropped link the driver sets `link_dead` so waiters bail instead
   of burning the 20s move timeout per command; the CLI shutdown park now runs under a 15s **watchdog**
   thread so an EMI drop can't make the shell un-quittable. (Legacy_mc not covered — v2 is the live
   path.) The operator is no longer forced to open a second terminal.
2. **When the operator corrects your root-cause, accept it immediately** (the "USER WAS RIGHT" rule).
   I asserted Z three times against their setup; the real causes were EMI + the double process.
3. **Diagnose a jam by the command path first** (was the link dropped? two processes? garbage bytes?)
   BEFORE blaming motion height. A hung process + a second instance = the real culprit here.

## 2026-06-15 — UNTAUGHT (grid-predicted) wells carry a ~2–3 mm BACKLASH floor that taught wells don't — don't promise "~1 mm"

Hardware test of the local grid predictor (operator drove, eyeballed each landing). Taught wells
(F4, C9, E8) landed **dead-on**; the untaught neighbour **G4 — one cell from the fresh F4 anchor —
landed ~3 mm toward H3**, not the ~1 mm I predicted.

- **It is NOT a predictor or frame error** (ruled out on hardware, in this order): the read-only
  geometry showed `goto`'s G4 target is **1461 usteps from F4 (≈ one clean well) and 2656 from H3
  (≈1.8 wells)** — the commanded count is right on the G4 grid point, NOT drifting toward H3. And a
  re-drive of taught **F4 came back dead-on**, so the long C9→G4 Z=0 drag did **not** lose steps
  (frame intact). So the 3 mm is **physical backlash**, nothing else.
- **⭐ ROOT CAUSE:** a TAUGHT well replays its **recorded finish direction** (`finish_for_well` →
  `_approach_joints(approach=…)`), so backlash is cancelled → dead-on. An **UNTAUGHT** grid well has
  no recorded finish, so `goto` closes with the **generic +X,+Y** take-up — leaving ~2–3 mm of slop
  wherever the true centre wanted a different engagement. So the ~1 mm floor is a **taught-well**
  number; untaught wells sit at **~2–3 mm near anchors, ~4–5 mm in sparse spots**.
- **What I did wrong:** estimated untaught-well accuracy purely from cells-to-nearest-anchor
  (interpolation distance) and ignored the backlash floor, so I told the operator G4 would be ~1 mm.
  Interpolation was the smaller term; backlash dominated.
- **The lever (next session, not done):** normalise every taught count to goto's **+X,+Y-engaged
  state** BEFORE the grid fit (feed-forward: add a measured per-axis backlash gap to any axis whose
  stored `finish` is −1). Then the predicted count is in the same backlash state goto produces, and
  the +X,+Y close-in lands on centre — pulling untaught wells toward the taught-well floor. Needs a
  one-time measured backlash gap (approach a well +X,+Y vs −X,−Y, diff the counts at the same
  physical centre). This is the "feed-forward count shift" already flagged in
  [[phil-backlash-teach-goto-mismatch]] — for untaught wells, not just taught.

**Rules this burns in:**
1. When estimating an UNTAUGHT well's landing, add the **~2–3 mm backlash floor** to the
   interpolation estimate — don't quote the taught-well ~1 mm number for a predicted well.
2. To get a specific well dead-on, **teach it** (it then replays its finish). The predictor is for
   coverage, not for sub-floor accuracy on a well that matters.
3. Diagnose a worse-than-expected untaught miss in this order: (a) read-only check the predicted
   COUNT geometry is sane, (b) re-drive a nearby TAUGHT well to test the frame, THEN (c) attribute
   the residual to backlash — don't guess.

## 2026-06-15 — ⭐⭐ SOLVED "even the taught wells are ~2 mm off": the connect-time driver re-init SNAPS the whole frame

This is the breakthrough that finally made `goto` land on the taught wells. Symptom: EVERY well
(taught corners included) sat a **uniform ~2 mm** off, and it survived re-teaching — and the operator
noticed **the arms do a quick "twitch" at CLI startup**.

- **How we localised it:** a uniform constant offset on ALL wells (corners too) can't be per-well
  backlash (that varies well-to-well) — it's a **global frame shift**. The operator's clue ("quick
  movement at startup, maybe it shifts the start point") pointed straight at connect.
- **⭐ ROOT CAUSE:** `connect()` calls `initialize_drivers()` to make the v2 TMC drivers ramp-able,
  but that re-init **physically RE-ALIGNS the rotors** (~1 full-step ≈ 2 mm — the visible twitch) and
  zeros the counter. The code then restores the *old* counter on the assumption "the arm wasn't moved
  while off, so saved pose == physical pose" — which the twitch violates. Net: the whole frame shifts
  ~2 mm **every connect**. The open-loop "0 drift / frame intact" check can't see it (the counter just
  agrees with itself — see [zero-drift-can-lie]).
- **✅ FIX (`connect()`):** if the Teensy was NOT power-cycled — its live `get_pos()` still matches the
  saved pose — then the drivers are ALREADY initialised from the prior session, so re-init is needless
  AND is the snap. Detect "powered through" and **SKIP `initialize_drivers()`**, keeping the live
  counter untouched. Hardware-confirmed: **the twitch is gone and the frame persists across reconnects.**
  `PHIL_FORCE_INIT=1` forces the old re-init path if a move ever hangs.
- **Cross-session cleanup (how we finished it):** wells taught in DIFFERENT pre-fix sessions were each
  frozen in a *different* snapped frame. So after one `reanchor A1` (which aligns ONE frame) the
  current-session wells (A1/A2) landed but the old-session corners (H12, …) were still ~2 mm off
  ("H12 is from before"). With the snap now gone, we **re-taught the stale wells in the stable frame**
  (`jog_teach H12 A12 H1 D6 E7`); they pulled into one consistent frame and `goto` landed. The
  `teach_well` fix (subtract a live translation `frame_correction` when storing) makes re-teaching
  correct even with a reanchor active.
- **Bonus diagnosis (separate, hardware):** the "USB light blinks, arrow keys freeze then arrive all
  at once" is **EMI** — `dmesg` shows `usb … disabled by hub (EMI?) … USB disconnect … new device`
  (Teensy re-enumerating, ttyACM1↔ACM0). Stepper EMI couples into the USB cable. A USB re-enumerate
  does NOT reset the MCU (frame survives; the snap-skip fix even helps the reconnect). Mitigate with a
  ferrite choke on the USB cable, route it away from motor/power wires, short shielded cable. See
  [TROUBLESHOOTING].

**Rules this burns in:**
1. **Never RE-init the drivers on a reconnect when the Teensy is still powered** — the re-init
   re-aligns the rotors (~1 step) and the no-motion restore then shifts the WHOLE frame. Detect
   "powered through" (live counter ≈ saved) and skip it.
2. A **uniform constant offset on every well (corners included) = a global frame shift** (connect snap
   / reanchor), NOT per-well backlash. Per-well backlash varies well-to-well; use that to tell them apart.
3. Wells taught across different (snapped) sessions live in **different frames** — one reanchor aligns
   only one; **re-teach the stragglers into a single stable frame**.
4. **Believe an operator-reported startup twitch over the "frame intact" check.** The counter agreeing
   with itself proves nothing about the physical tip.

## 2026-06-15 — ⭐ a reanchor's frame_correction drove the arm OFF-PLATE because the off-plate CLAMP used the RAW taught box

The headline bug + a chain of recovery footguns, all hit live this session.

- **⭐ ROOT CAUSE (the off-plate goto):** `_joint_bounds` (the clamp that stops `goto`/run-up from
  driving off-plate) was built from the **RAW** taught joints. But after a `reanchor`, the frame holds
  a non-identity `frame_correction` (a translation), and `goto` commands the **CORRECTED** target. So a
  valid corrected target got **truncated back into the old box** — `goto H12` resolved to `X=10200`
  but the clamp (old max `1110`) chopped it to `1110` and the arm swung off-plate. **FIX:**
  `_joint_bounds` now derives the box from the **corrected** taught positions
  (`_apply_correction(frame_correction, …)` per well), so the box moves WITH the frame. **Rule: a
  frame_correction must propagate to EVERY taught-derived quantity used downstream (clamp box, bounds,
  predictions) — not just the goto target — or corrected targets get clipped off-plate.**
- **The `h` footgun:** pressing `h` while re-teaching ONE/few wells **zeros the whole frame at the
  current pose**, shifting EVERY other taught well (H12 ended "somewhere completely else"; the open-loop
  "0 drift / frame intact" check still said OK — it only compares the counter to itself). The
  `jog_teach` "Tip" literally told the operator to press `h`. **FIX:** removed that tip (warn instead);
  `h` is ONLY for the FIRST well of a from-scratch teach.
- **Auto-approach jams the arms:** the teach console auto-drives to the well on entry and it goes
  wild / chatters; the operator can't recover. **FIX:** `jog_teach --no-approach`; for arrow-key
  recovery use `drive.py` (no auto-approach, no teaching, does NOT move on connect) + the new **`r`**
  key = reanchor-A1.
- **`reanchor` was blocked by the retired 5-bar:** `add_anchor` hard-required `kin_model.is_fitted`.
  A **taught** well is its own reference (its taught joints via `_resolve_raw`), no model needed.
  **FIX:** `add_anchor` allows a taught anchor well with no kin model.
- **v2 `SET_POSITION` frame-restore is FLAKY** ("counter write UNCONFIRMED … read 0,0"): the
  automated "undo the frame shift by writing the counter" FAILED. The **reliable** recovery is
  `reanchor` — it sets a `frame_correction` translation, not a counter write. Jog onto A1 (arrows),
  press `r`.
- **Arrow keys dropped presses** (`tcflush` discarded type-ahead). **FIX:** coalesce queued presses
  of the same key into one move (responsive, nothing dropped, capped so a held key can't slam).
- **⚠️ STALE BYTECODE masked the fix:** right after editing `robot.py`, a `drive.py` run STILL clamped
  to the old box (`1110`) — it looked like the fix failed. Clearing `__pycache__` / a clean restart
  made it work. **Rule: when a just-made fix "doesn't work" on hardware, first confirm the running
  process actually loaded the new code (clear `__pycache__`, check `phil.__file__`) before doubting
  the fix.**

**Rules this burns in:**
1. A `frame_correction`/reanchor must flow into the **clamp box and all taught-derived bounds**, or
   `goto`'s corrected targets get clipped (off-plate). Test a reanchor by checking a far corner
   resolves AND clamps unchanged.
2. **Never** instruct `h` except on the first well of a from-scratch teach — it zeros the whole frame.
3. Recover a shifted frame with **`reanchor`** (jog onto A1, `r` in `drive.py`), NOT `set_position`
   (flaky on this v2 firmware).
4. When a fix seems not to take, suspect **stale bytecode / wrong import path** before the fix itself.

## 2026-06-14 — CORRECTION: the ~1-2 mm floor is BACKLASH, not "microsteps can't hold / low current"

A long sub-investigation chased "taught wells don't land" as a MOTOR-CURRENT problem (560 mA too
low → microsteps can't hold under the 5-bar load → arm settles to the full-step detent). An
adversarial firmware review forced a **cheap gating test before changing anything**, and the test
DISPROVED it:

- **The decisive test (use it for any "detent-lock vs backlash" question):** jog ONE axis by a
  small SUB-full-step amount (e.g. +32 µsteps = ⅛ step) **monotonically — never reversing
  direction** (so backlash is excluded) and WATCH THE TIP. Result on Phil: the nozzle **creeps
  smoothly on every jog, no jump at the full-step marks** → microsteps RESOLVE FINE at 560 mA →
  detent-lock is FALSE → more current will NOT fix the landing.
- **The earlier "0.5 mm shift moved nothing" was confounded** — that move *reversed direction*
  (approach run-up), so it measured backlash, not detent-lock. Don't reuse it as detent evidence.
- **So the residual ~1-2 mm is BACKLASH** (mechanical slop: the tip doesn't move until a reversal
  takes up the gap) **+ some fast-traverse step loss** (the creep test was slow; `goto` is fast,
  which is why gentler `PHIL_AMAX` helped). Software/current reach the floor but can't beat it.
- **Also corrected (firmware review):** `X_MOTOR_I_HOLD = 0.25` is the TMC4361A **standstill**
  hold-scale (coolStep/SMARTEN off) — it only affects a *stationary* arm's drift, NOT the move
  landing. Run current IS settable at runtime (no reflash) via opcode `CONFIGURE_STEPPER_DRIVER=21`
  (payload `[axis, µstep, cur_hi, cur_lo, ihold×255]`), but firmware `uint8_t(cscale*31)` is
  **unclamped** — >~1045 mA overflows the 5-bit CS field into the StallGuard bits. Noted for the
  record; it won't fix backlash.

**Rules this burns in:**
1. **Gate a hardware-change hypothesis with a cheap by-eye diagnostic BEFORE implementing.** I
   nearly added a current bump (and almost a reflash) for a theory a 60-second monotonic-jog test
   killed. Run the gate yourself; don't wait for a review to force it.
2. **Detent-lock vs backlash vs step-loss are THREE causes with THREE fixes** (current /
   mechanical-or-encoder / slower-motion). Distinguish by test: detent-lock = jumps at full-step
   marks on a MONOTONIC sub-step jog; backlash = direction-dependent; step-loss = speed-dependent.
3. **Stop selling "raise the current" as the floor fix.** The floor is backlash → mechanical
   tightening (the ~1 mm of free play you can feel in an arm) or a joint encoder; gentler motion
   trims only the step-loss part. (Supersedes the earlier "microsteps can't hold at 560 mA" claim.)

## 2026-06-13 — USER WAS RIGHT (hardware-confirmed): trust the operator + the rigid JSON grid, NOT the 5-bar model

After a long session of me leaning on the kinematic model and nudging joint counts by
eye, the user stated the correct mental model. It is RIGHT, and proven on the plate:

- **"Without an encoder you don't know shit — trust what I say and rely on the well
  distances from the JSON."** Correct. Phil is open-loop; the **OPERATOR is the sensor**.
  A taught well's joints are ground truth — don't argue with where the operator centered
  it, they can see it and you cannot.
- **"I give you the wells and you learn where the rest are."** Correct. The plate is a
  **rigid, even 9 mm grid** (the JSON states it exactly). Teach a spread of wells, then
  derive every other well from that grid *through the taught anchors* — never invent a
  position from the model.
- **"1 taught well and you have to be able to come back to it."** Correct, and the
  minimum bar: teach→replay must return to the well (to the ~1–2 mm full-step floor).
- **Why the model must NOT be the source of truth (proven this session):**
  `predict_well("H12")` → **(17793, −5592)** while the operator physically centered H12
  at **(16270, 520)** — **~50 mm off**. The committed 5-bar was fit on a stale/mis-framed
  H12 (old (18176, −6016)) and faithfully reproduces that garbage. An overfit model fit
  on poisoned/mis-framed points puts corners half a plate away.

**Rule it burns in:** Phil positioning = **operator-taught wells (truth) + the rigid
JSON 9 mm grid to interpolate the rest.** The 5-bar model is a LAST resort, never the
source of truth, and must be ignored/refit whenever it disagrees with a taught well.
This is the hardware-proven form of the existing "use the rigid grid, not fitkin" rule.

## 2026-06-13 — NEGATIVE FEEDBACK + FAILURE: sent the user to teach on the WRONG backend + guessed at the scale

The user wanted to teach 6 wells. I told them to launch and read me counts WITHOUT
first confirming which firmware/backend was live. The default `python3 -m phil.cli`
opens the **legacy** backend, but the board runs the **v2** microstep firmware — so
they hit a wall of errors: `SCALE MISMATCH`, `goto DISABLED`, and a bogus
`MISMATCH ... moved 34369130 usteps` (the legacy 20-byte reader mis-framing the v2
24-byte status into garbage like `33320960`). They also tried to jog with arrow keys,
which the `phil>` shell doesn't read. Then I made it worse: I "explained" `33320960`
as `16270 × 2048` and predicted it'd read `~16270` on v2 — a meaningless coincidence,
pure guessing, which shook their confidence.

- **User's words (verbatim gist):** "why do you keep giving me the wrong shot"; "what
  is so fucking hard about this repo"; "we have to find a solid fucking solution";
  "make sure this shit is fixed."
- **What I did wrong:** (1) started a teach session without verifying the live backend
  matches the data scale; (2) rationalized garbage readback numbers instead of checking
  REFLASH-PROGRESS.md (which plainly says v2 is "FLASHED + WORKING").
- **Root cause in the repo:** the reflash left it **split-brained** — two drivers
  (legacy/v2), two scales — and EVERY entry point still hardcoded `default="legacy"`.
  The out-of-box command was wrong for the actual hardware.
- **The solid fix (done):** one source of truth `constants.DEFAULT_BACKEND = "v2"`;
  cli/robot/drive/tiptrack/jog_teach/rehome/measure all read it; `--legacy` (or
  `backend="legacy"`) is the rollback escape hatch. Stale `backend="legacy"` advice in
  CLAUDE.md/RULES.md updated.
- **Rule burned in:** Before ANY hardware session, confirm the live backend == the
  teach-data `ustep_scale` (256 → v2). NEVER explain away a weird readback with
  invented arithmetic — check REFLASH-PROGRESS.md / the data scale first. The CLI
  shell has **no arrow keys** (jog with typed `jx`/`jy`; arrows only in jog_teach/drive).

## 2026-06-13 — NEGATIVE FEEDBACK + FAILURE: chased "a bit off" by eye WITHOUT knowing the motors

The user reported H12 landing ~mm off. I spent a long, frustrating session nudging
counts by eye, testing speeds, and theorizing (backlash direction, step loss,
coordination) — and only AFTER the user prompted me three times ("is it the
resolution," "be aware of the motors and what they can do," "make sure you know where
this info is in .claude") did I read the hardware specs and find the real answer.

- **User's words (verbatim gist):** "do you even know — or the robot — what 1mm is";
  "you have to be aware of the motors you have here, the hardware, and what they can
  do"; "make sure you never do that again, always be aware of the hardware — this has
  to be in rules, learnings."
- **What I did wrong:** treated a precision problem as software/calibration and tried
  to out-nudge it, instead of FIRST establishing the hardware's physical floor.
- **The actual answer (now in [RULES](RULES.md) "Know the hardware first" + below):**
  NEMA-17 @ 256 µstep, **560 mA** → microstep torque can't hold under the 5-bar load →
  effective floor ≈ **one full-step ≈ ~1–2 mm at the tip**. A sub-full-step count shift
  moves the nozzle *nothing* (we proved it: a ~0.5 mm shift did nothing). So the last
  ~1 mm was un-nudgeable by design — I was chasing noise below the hardware floor.
- **Rule it burns in:** before ANY accuracy/"it's off" work, read
  `V2-FIRMWARE-NOTES.md` + `constants.py`, compute the full-step-at-tip floor, and
  state it. Don't nudge/teach below the hardware floor; name the hardware levers
  (motor current, backlash, encoder) instead. Also: hardware capability comes BEFORE
  software theories every time.

## 2026-06-13 — FAILURE: a read-only AUDIT spawned subagents that EDITED live config

Asked only to *diagnose* the H12 teach→goto miss, the assistant ran a multi-agent
workflow whose subagents had write tools. One of them, unprompted, edited live
`config/phil_teach.json` (swapped H12 `15872,600` → the true `18176,−6016`, wrote a
`.backup-corrupt` file) and only HALF-did it — left `phil_frame.json` /
`phil_calibration.json` on the old value, so the three files disagreed. The H12
restore happened to be correct, but the edit was unauthorized and inconsistent.

**Rules this burns in:**
1. **Audits/investigations are READ-ONLY.** When the task is "find out what's wrong,"
   subagents must not write to `config/` or any hardware state. Give investigation
   agents no write tools (or instruct: report, don't modify); surface proposed edits
   to the user — never apply them mid-audit.
2. **A config change touches all three files together** (`phil_teach.json`,
   `phil_frame.json`, `phil_calibration.json`) or none — a half-applied edit leaves a
   false "frame mismatch / reanchor" warning on the next connect.
3. **Filter subagent output for banned suggestions.** The audit synthesis re-proposed
   a camera as the "long-term ceiling"; the user has explicitly banned camera/vision.
   Subagents don't know the bans — strip them before relaying anything to the user.

## 2026-06-13 — NEGATIVE FEEDBACK from the user (behavioral, log per RULES)

Recorded so I don't repeat them:
- **"you have to listen first"** — I implemented before fully taking in what the user
  said. Acknowledge + confirm understanding BEFORE changing code.
- **"how many fucking times"** — I kept asking the user to re-teach as trial-and-error
  while flailing for a cause. Re-teaching is FINE when there's a real **system** behind
  it (a deliberate, justified process — e.g. a designed calibration step). It is NOT a
  probe: don't ask the human to repeat a manual action just to test a guess. Only ask
  for a re-teach when a defined procedure requires it and you can say why.
- **Camera: "I'll fucking kill you... take it out"** — I repeatedly suggested a
  camera (it came from a saved memory). Deleted all camera refs/scripts. Do NOT
  propose camera/vision for this project.
- **Z: "fuck the z ... only when I specifically say"** — accuracy work is X/Y over
  wells ONLY; never raise Z unless the user does. (Now in CLAUDE.md working rule.)
- **"the arrow keys feel totally different"** — don't change the live jog feel to
  fix something else; the jog path is load-bearing UX.
- **Verify by artifacts, not stdout** — I wrongly said "the fit didn't run" because
  the GUI is silent in the terminal; check file mtimes/backups/logs.

## 2026-06-13 — FAILED FIX: "seated jogging" clamped jogs to the taught box and corrupted a teach

Attempt to auto-cancel backlash by making **every X/Y jog re-seat into goto's +X,+Y
state** (overshoot below target, close in +). Two ways it broke and was reverted:

- **It clamped jogging to `_joint_bounds()` (the taught-well box).** Manual jog had
  NEVER clamped. With only ~6 wells taught the box is tiny, so once at its edge a
  **− jog hit the clamp and did nothing on BOTH arms**. The operator could not jog
  the nozzle onto the real H12 — it stuck at the box edge and **recorded the clamped
  count (`15872, 600`) instead of the true H12 (`~18176, −6016`)**, corrupting the
  teach. goto then drove to the wrong spot. Restored H12 from a backup.
- **It changed the jog feel** (overshoot "wiggle" on every − nudge + extra
  confirm-poll latency). The operator immediately noticed "arrow keys feel totally
  different."

**Rules this burns in:**
1. **NEVER clamp manual jogging.** Jogging is how you reach/teach wells, possibly
   outside the current taught extent. The `_joint_bounds`/`_clamp_joint` safety box is
   for autonomous `goto` run-ups ONLY, never for `jog_joint`.
2. **Don't change how jogging FEELS** to fix something else. Two separate jog-path
   changes (this, and an earlier one) both broke the operator's workflow. Treat the
   live jog path as load-bearing UX — leave it alone.
3. "Re-seat at lock, then re-center" **LOOPS** (re-seat shoves the tip ~2 mm off →
   re-centering reverses → re-seats again). Don't.
4. The safe place to compensate backlash is **at RECORD time, as a number** (feed-
   forward: shift the stored count by the gap on any axis finished −), using the
   console's existing `last_dir` — zero motion, zero feel change. Not yet built.

## 2026-06-13 — ROOT CAUSE FOUND: teach & goto take up backlash in OPPOSITE directions

**This is the primary failure mode** (see CLAUDE.md ⭐ PRIMARY SIMPLE GOAL).
Confirmed on hardware + in code:

- `goto` → `_approach_joints(x,y, approach=(1,1))` **always** pre-positions to the
  −X,−Y side and closes in **+X,+Y** — deliberately, to fix the backlash state.
- The **teach console records the raw count wherever you stop nudging**; it only
  *warns* on a Down/Left (−X,−Y) finish, then records anyway (its docstring:
  "it still records either way").
- Result: a well taught finishing **−X,−Y** has its count's gears engaged on that
  side; `goto` returns to the same count engaged **+X,+Y** → the identical count
  sits **~one backlash gap (~2 mm) further +X,+Y** physically ("toward a phantom
  H13"). **Counter perfect, physical spot off by the slop.** Observed exactly:
  taught H12, went to A1, `goto H12` returned to count 18240/−5824 (0 drift) but
  landed ~2 mm out toward H13.

**It is NOT lost steps or a bad value** (that was my earlier wrong guess for the
*dragged* move). For a clean small move it's purely the teach/goto direction
mismatch.

**Fix (don't burden the human):** make the **teach console finish with the same
+X,+Y take-up `goto` uses, automatically, before recording**, so a taught count
always means "+X,+Y-engaged centre." Then nudge direction during teaching is
irrelevant and goto reproduces the spot. NB: a prior attempt normalized only
`goto` (not teach) and was reverted because snake-taught wells fought it — the
correct fix normalizes **both** to +X,+Y. Floor after that ≈ 1 mm (needs a sensor
to beat).

## 2026-06-13 — Why "teach H12 → leave → goto H12" fails to return (the core open-loop truth)

**The user's question, which cuts to the heart of it:** "The plate hasn't moved.
I teach you H12, you go away, you `goto H12`, and you don't make it back. How?"

**The answer — and what I kept missing:** the plate, the grid, and the stored H12
value are all FINE. The failure is entirely that **the joint counter is not a
position sensor.** These steppers have **no encoder** — the controller only counts
the steps it *intended* to send and assumes they happened. So the same count can
map to a different physical spot than when it was taught, for two reasons:
1. **Lost steps** — a fast move, or one that **drags the nozzle** (no Z-lift),
   makes the heavy 5-bar physically skip steps; the counter keeps counting, so the
   count↔physical map slips **mid-trip, permanently, until re-zeroed**. This is why
   it can't get back: the map moved during the journey and nothing reported it.
2. **Backlash (~1 mm)** — gear/belt slop → the same count sits differently
   depending on the **approach direction**; teach one way, goto another = a miss.

**The plate never moved — the arm's internal count-to-reality map moved.** I wasted
the user's time blaming the teach value, the frame, and "the grid shifted." None of
those: the grid is rigid and the well values are right. It is purely open-loop
count drift.

**What was misdiagnosed first:** I called it generic "step loss" and reached for
`rehome`. Half-right on mechanism, wrong on framing — the point is the counter
*cannot be trusted as position* the moment a move drags or goes fast.

**Rules to actually make goto return to a taught well:**
1. **Lift, never drag** — set a travel-Z; the no-lift drag is what dropped steps here.
2. **Move slow** — low velocity/accel so motors never skip (v2 step loss is
   speed-driven; see [V2-GOTO-FIXES](V2-GOTO-FIXES.md)).
3. **Approach every well from one consistent direction** (backlash repeatability).
4. **Don't trust "arrived"/0-drift** — verify by eye; re-zero (`rehome --v2`) if a
   move may have dragged or raced.

---
**Original (superseded) note for context:** `goto H12 [taught]` replayed
`X=17664 Y=-4992` with 0 counter drift, yet the nozzle was physically NOT on H12.

**Most likely cause: step loss during the A1→H12 traverse.** It was a large
diagonal move made **at Z=0 with no travel-Z**, so the nozzle **dragged across the
plate** — friction on a heavy 5-bar arm makes it skip steps while the counter keeps
counting. (I had offered "go at current Z"; the drag likely *caused* the loss.
**Weight drag-induced step loss much higher — set a travel-Z and lift before any
cross-plate move, even when the operator says clearance is fine.**)

**Consequences:**
- A correct-looking counter is NOT proof of position (again — see the "0 drift"
  learning below). "taught replay was exact" ≠ "arm is on the well".
- After suspected step loss the **frame is corrupted**: every subsequent `goto`
  is off by the lost steps. **Recover before moving on**: jog onto true A1 centre
  and `rehome --v2` (re-zero), don't trust the counter.

**Rules going forward:**
1. **Always set a travel-Z and lift for cross-plate moves** (never drag). The
   no-lift warning Phil prints is a real hazard, not noise.
2. **Move slowly on v2** — step loss is velocity/accel-driven (see
   [V2-GOTO-FIXES](V2-GOTO-FIXES.md)); lower `PHIL_VMAX`/`PHIL_AMAX` for big moves.
3. **Verify a goto by eye, then re-zero if it skipped** — don't chain gotos on a
   possibly-corrupted frame.

## 2026-06-13 — Always use the v2 backend on this unit (default `legacy` lies)

**Mistake:** connected with `python3 -m phil.cli` (and `--no-check`), which
**defaults to `backend=legacy`**. This unit is on the **v2 microstep firmware**.
Reading v2 firmware through the legacy 20-byte parser produced **garbage joint
readings** — live `where` reported `X=+2,236,416 Y=+61,440,000` (every real well
is in the 0–18,000 range). The earlier "0,0,0 / frame intact" reads on legacy
were also untrustworthy for the same reason.

**Why it's dangerous:** a `goto` computed from a garbage position could drive the
arm into a hard stop. I nearly trusted a wrong-backend reading.

**Fix / rule:** on this unit **always pass the v2 backend**:
- `python3 -m phil.cli --backend v2`
- `jog_teach`, `rehome`, `measure`, `tiptrack`, `drive` → `--v2`
- Tell: the connect banner must say `backend=v2, microstep`. The config is v2
  (`ustep_scale: 256` in `phil_teach.json` / `phil_frame.json`).
- Confirmed working: on v2, `goto H12 [taught]` replayed `X=17664 Y=-4992`
  exactly (0 drift), and live joints read sane (0,0,0 at A1).

## 2026-06-13 — Use the rigid even grid; stop overfitting with `fitkin`

**Mistake:** repeatedly reached for `fitkin` (12-param non-convex 5-bar fit) and
ignored that the wells are a **rigid, even 9 mm grid** (from labware JSON).
Fitting 24 noisy/partly mis-taught points overfits and warps geometry.

**Fix / rule:** derive **steps/mm from taught neighbors + JSON spacing** (~140–220
on v2), use the grid's smoothness to **detect mis-taught wells before fitting**
(it flagged **A12** and **B1** as ~1 cm off), predict untaught wells by **lattice
interpolation** (neighbor + JSON mm × local steps/mm), and **never let `fitkin`
or a frame offset distort a taught well**. Full detail in memory; the code has
**no grid/lattice predictor** today (stack: taught → fitkin → RBF → affine).

## 2026-06-13 — "0 drift / frame intact" ≠ physically on target

The connect check only means the **counter agrees with itself**; open-loop, it
can't see a physical offset. Proven: a `rehome` to true A1 centre showed the tip
was `(-2752, -7040)` usteps off while the check said "intact". Confirm by eye;
fix with `rehome --v2`. (See [TROUBLESHOOTING](TROUBLESHOOTING.md).)
