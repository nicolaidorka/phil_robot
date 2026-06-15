# Phil v2 (microstep) firmware — hard-won notes

Everything learned bringing up the octopi_firmware_v2 microstep reflash on Phil (the Squid V4
board: Teensy 4.1, TMC4361A motion controller + TMC2660 drivers). Read this before touching the
v2 firmware or the v2 connect path. Companion: [REFLASH-PROGRESS.md](REFLASH-PROGRESS.md).

## The big picture
Phil is open-loop (no encoders). The joint position lives ONLY in the TMC4361A `XACTUAL`
register — volatile, lost on any power loss. There is no EEPROM. Reliable teach-and-replay
therefore needs the HOST to persist the joint frame and restore it on connect. We added a
firmware `SET_POSITION` opcode for that.

## Three gotchas that cost us a day (in dependency order)

### 1. INITIALIZE is REQUIRED for the motors to move — boot init alone is NOT enough
The `setup()` driver init at boot leaves the TMC4361A unable to *ramp* — motors hold but won't
jog/move. Only the host **INITIALIZE (opcode 254)** makes them movable (it re-runs
`tmc4361A_tmc2660_init`). Empirically proven: a raw jog after boot does nothing; the same jog
after `INITIALIZE` moves. **So the v2 connect MUST send INITIALIZE.** (This is why teaching
worked originally — the old connect sent INITIALIZE — and broke when we removed it.)

### 2. ...but INITIALIZE zeros the counter — so restore the frame right after
`INITIALIZE` (via `tmc4361A_tmc2660_init`'s chip reset `0x52535400`) clears `XACTUAL`. So connect
does: **INITIALIZE → SET_POSITION(saved X/Y/Z) → velocity command.** Net = movable motors AND a
preserved frame. (`robot.py` v2 connect block.)

### 3. SET_POSITION must NOT use `tmc4361A_setCurrentPosition` — it traps the axis
The library's `setCurrentPosition` writes `VMAX=0` and sets `velocity_mode=true`, leaving the axis
stuck out of position mode (couldn't ramp afterward; only an INITIALIZE chip-reset recovered it —
X/Y got hit, Z happened to survive). Our firmware `SET_POSITION` handler instead does, per axis:
`writeInt(VMAX,0)` → `writeInt(XACTUAL,v)` → `writeInt(X_TARGET,v)` → `tmc4361A_sRampInit()` →
`velocity_mode=false`. That sets the counter with **no motion** (VMAX=0 during the write,
XACTUAL==XTARGET) and leaves the axis **movable** (sRampInit restores VMAX from rampParam +
forces POSITION mode). `set_home` on v2 also routes through SET_POSITION for the same reason
(HOME_OR_ZERO_ZERO hit the same `setCurrentPosition` trap).

## The working v2 connect sequence (robot.py)
1. `sleep 0.3`
2. (optional) read `get_pos()` for a bump hint vs the saved frame
3. `initialize_drivers()` (INITIALIZE) — motors movable, counter zeroed; `sleep 0.8`
4. restore: `set_position_usteps(X/Y/Z, saved)` if a saved frame exists; `sleep 0.05`
5. `set_max_velocity_acceleration` X/Y/Z (snappy jogging; firmware applies live)
6. `_check_frame()` (A1 habit)

## Frame persistence design
- Host saves the live joint frame on EVERY jog (`jog_joint`), goto, `set_home`, `fit_anchor`,
  and `close` → `config/phil_frame.json` (`last_x/last_y/last_z` + `ustep_scale`).
- `jog_joint` and `set_home` saving the frame is CRITICAL — teaching is all jogging; without it
  the saved checkpoint is stale and the restore sets a wrong position.
- The frame file is scale-tagged: on v2, a frame written at v2 scale (`ustep_scale==256`) is kept
  & restored; a legacy-scale one is dropped.
- Firmware opcode: `SET_POSITION = 19` (free). Payload `[axis, int32 BE microsteps]`. Host:
  `v2_mc.set_position_usteps(axis, microsteps)`, `expect_ack=False`.

## Hardware facts (measured/confirmed)
- **+Z = UP.** Z scale ≈ **8,900 microsteps/mm** (the def_phil.h Z pitch 0.012*25.4 is NOT Phil's
  real Z lead — harmless, we command microsteps directly; only the mm-convenience path is notional).
- X/Y ≈ **170–227 microsteps/mm at the tip** (varies with the 5-bar Jacobian; right/high columns
  move ~5 mm per legacy full-step, col-1 region ~2 mm — col-1 is the most precise region).
- **Axis map** (def_phil.h): `x=1, y=0, z=2` (X/Y swapped vs natural). Verified by jog; X command
  drives channel 1 consistently (set/read/move all agree). NB: this is the **low-level firmware
  channel** mapping, handled inside the driver — it does **not** change the user-facing convention
  (logical **X = left arm, Y = right arm**) that CLAUDE.md/ARCHITECTURE.md use; jog/teach/goto all
  speak the logical axes.
- Motor current: X/Y **560 mA**, Z **300 mA** (TMC2660 current-scale ≥16). Motors: Shinano SST43D1125 NEMA-17.
- No motion on connect; `SET_POSITION` changes the counter without moving the arm (verified).

## Teach workflow on v2 (do it in ONE session)
1. `python3 -m phil.jog_teach --v2 A1` (or `--all`). Jog to A1 (arrows; `-` for fine steps,
   down to ~0.035 mm). The v2 jog steps are `[8..2048]` microsteps.
2. Center A1, press **`h`** (`set_home` — clean zero, now saved properly), **Enter** to record.
3. Teach more wells in the SAME session. `s` saves, `q` quits. The frame now persists across
   reconnects and power-cycles (no re-anchor needed unless the arm was hand-moved while off).
4. `fitkin` after enough wells to build a v2-scale 5-bar model.

## Precision reality (the actual goal)
- Hard floor ≈ **1 mm backlash** (open-loop, mechanical) — the reflash removed the 3–5 mm command
  grid but not backlash.
- 4 corners + bilinear/RBF interpolation → a few mm mid-plate. For **reliable** well-hitting,
  teach the BOUNDARY (rows A–E + H), refit, interpolate F/G (the original strategy). 96-well
  (9 mm pitch) is forgiving; 384-well (4.5 mm) needs the boundary teach + the ~1 mm floor to hold.
- **Current v2 state (2026-06-13): only 24/96 taught** (an L-shape: row A + col 1 + a few) —
  the boundary teach above is **NOT yet redone in v2**, so untaught wells extrapolate (corner
  LOO ≈ 12–13 mm). This is the open work to "know the wells." See
  [UNITS-AND-CALIBRATION](UNITS-AND-CALIBRATION.md) and the main [CLAUDE.md](CLAUDE.md) Goal/Status.
- Always close in from a FIXED direction (the `_approach_joints` run-up) for repeatable backlash.

## Operational gotchas
- The `jog_teach` console **auto-approaches the target well before the arrow keys go live** — if
  that move stalls (e.g. a stuck axis), the console looks frozen (no keys register). Kill it with
  `pkill -9 -f jog_teach` from another terminal; the cbreak terminal may need `reset`/`stty sane`.
- Flashing: if "Teensy did not respond to a USB request to enter program mode" — press the
  physical PROGRAM button on the Teensy (USB-connector end). Auto-reboot can fail if the running
  firmware is in a bad/busy state.
- One connection holds the serial port — quit `jog_teach`/CLI before running a separate script.
