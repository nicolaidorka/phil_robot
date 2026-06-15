# Troubleshooting — Phil

Symptom → cause → fix. Failure modes that actually happen on this unit. Deeper
background is in [FINDINGS](FINDINGS.md); operating rules in [RULES](RULES.md).

## Connection

**`PhilHandshakeError` / won't connect.**
- Teensy unpowered or USB unplugged → check the cable; the frame is only
  preserved while the Teensy stays powered.
- Wrong device grabbed → it auto-detects by **SN `16640550` / mfr "Teensyduino"**,
  not a port number. The `/dev/ttyACMx` number is **not stable** (seen as both
  ACM0 and ACM1); don't hard-code it. An Opentrons Flex also shows up on a
  `ttyACMx` — auto-detect skips it. Confirm with `ls /dev/ttyACM*` and
  `udevadm info /dev/ttyACMx | grep SERIAL`.
- Port busy → another `phil` process still holds it; close it.

**USB light blinks, arrow keys/jogs freeze then "arrive all at once," port flips ACM0↔ACM1.**
- **EMI** from the stepper motors radiating into the USB cable. `dmesg` shows
  `usb usb1-port2: disabled by hub (EMI?) … USB disconnect … new high-speed USB device`
  (the Teensy re-enumerating). The serial link drops mid-jog so keypresses queue in the terminal,
  then flush at once when it reconnects.
- A USB re-enumerate does **NOT** reset the MCU → the joint counter/frame survives (and the connect
  snap-skip preserves it on reconnect); only the Python session breaks, so just restart.
- Fix (hardware): clip a **ferrite choke** on the USB cable near the Teensy; **route the USB cable
  away from motor/power wires** (cross at right angles, keep them apart); use a short shielded cable;
  try a different port; avoid hubs.

**Every command returns `CMD_CHECKSUM_ERROR` (status byte 2), then exits.**
- You're on the **stock** backend. This Teensy runs the **older 6/20-byte
  protocol**, not the repo's 8/24. Fix: `backend="legacy"` (the CLI default).
  Never use `backend="stock"` on this unit. (See FINDINGS → "Firmware protocol
  mismatch".)

## Position / frame

**Teach a well, then `goto` it misses — but the counter reads exactly on target.**
- The teach↔goto **backlash-direction mismatch**, now FIXED in code: teaching records
  the finish direction per axis and `goto` replays it (FINDINGS → "Anti-backlash, DONE
  RIGHT"). If a well still misses, it was **taught before the fix** (no finish stored →
  goto defaults to +X,+Y): **re-teach it** — center it whatever way is natural, press
  Enter, and goto reproduces it. No Up/Right discipline needed. Floor ≈ 1 mm backlash.

**Startup says "frame looks reset / suspect" (or asks you to reanchor).**
- A power-cycle or `reset()` zeroed the joint counter (geometry is still good).
  Fix: jog the outlet onto **A1** and `reanchor` (defaults to A1). **No re-teach.**

**Connect says "0 drift, frame intact" but the nozzle is visibly off the well.**
- Open-loop: that check only means the *counter* agrees with itself, **not** that
  the tip is physically right (no encoder). A bump that skipped steps mid-session
  is invisible to it. Fix: `rehome --v2` — jog to **true A1 centre**, ENTER zeros
  there, restores the whole taught frame. A `goto` cannot fix a physically-offset
  frame; only re-zeroing can.

**`goto <well>` lands off-centre / off-plate (especially v2).**
- v2 step-loss / too-fast notional velocity, off-plate run-up, coarse accept band
  — these were fixed; recovery is jog onto A1 + `rehome --v2`. See
  [V2-GOTO-FIXES](V2-GOTO-FIXES.md).
- If only the far edges are ~1 mm off and it's *systematic*, it's the model edge
  error, not a bug: center the 4 corners and `anchor A1/A12/H1/H12` → `anchor fit`
  (won't beat the ~1 mm backlash floor). Column 1 is the known-weak edge.
- If **one well** is off while its neighbours are fine, suspect a **mis-taught
  well** (bad jog/approach when it was recorded), not the model. Run
  `python3 -m phil.stepcheck` — it flags taught wells whose joint step-delta to
  their grid-neighbours is inconsistent. Re-teach the flagged well.

**`goto` warns "moving WITHOUT a lift — risk of dragging the nozzle".**
- No travel-Z set. Fix: jog Z up to clear the plate wall, then `travelz` (no arg
  captures the current Z as the safe height). Set it once per session before any
  cross-plate move.

**`goto A1` seems to hang / times out.**
- On legacy the move does settle-reads over serial that can block past a short
  timeout; if A1 is already at (0,0) there's nothing to move, so it's just
  bookkeeping. Give it more time, or read joints separately with `where`. Not an
  error by itself — confirm the joints afterward.

## Jog / teach consoles

**Arrow keys do nothing (jog/teach/measure/drive).**
- These need a **real interactive terminal** — a Bash tool call can't provide one
  (it hangs on stdin). Launch them in your own terminal with the `!` prefix.
- Right after launch the connect may still be finishing — wait for the live
  X/Y/Z prompt line, then press an arrow.

**I pressed `h` during `jog_teach --all` and wells moved.**
- `h` zeros the frame — it wrecks already-taught wells. Only ever press `h` on the
  FIRST well of a *fresh* full re-teach. To recover the frame without re-teaching:
  jog onto A1 and `reanchor`/`rehome`.

## Environment

**`scipy` ImportError / `well_map`/`kinematics` unavailable.**
- Optional dep. The package degrades to the affine fallback. Install with
  `pip install scipy` if you want the RBF map / kinematic fit.
