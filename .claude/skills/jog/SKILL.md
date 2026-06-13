---
name: jog
description: Drive the Phil arm with the keyboard arrow keys (free jog, no teaching). Use when the user wants to move/jog/drive the arm by hand with arrow keys, nudge an axis, or manually position the outlet.
---

# Jog the Phil arm with arrow keys

The user wants to move the Phil arm live with the keyboard. Arrow-key control needs
a real interactive terminal, which a Bash tool call does NOT provide — so do **not**
try to launch the driver yourself with Bash (it will hang reading stdin). Instead,
tell the user to launch it in their own terminal using the `!` prefix, which runs
the command in this session's live terminal.

## What to output

Give the user this ready-to-run command (the `!` makes it run in their terminal):

```
!cd /home/lundberglab/phil_robot-20250128T212609Z-001/phil_robot/software && python3 -m phil.drive
```

Add `--simulate` to try it with no hardware:

```
!cd /home/lundberglab/phil_robot-20250128T212609Z-001/phil_robot/software && python3 -m phil.drive --simulate
```

Then show the key map:

| Key | Action |
|-----|--------|
| Up / Down | X arm + / − (one whole step per press) |
| Left / Right | Y arm + / − |
| a / z | Z up / down |
| + / − | bigger / smaller jog step |
| g | go to a well (type the well id, e.g. `D6`, then Enter) |
| p | print the current joint position |
| q | quit |

## Notes to pass along

- `phil/drive.py` is a **safe** free-drive: it never teaches, never homes/zeros the
  frame, and never writes calibration — so moving the arm won't disturb the taught
  wells. To *record* wells instead, that's the teach console: `python3 -m phil.jog_teach`.
- Jog small — the joints are rotary with ~1 mm backlash; a few hundred usteps crosses
  the whole plate.
- The controller is auto-detected by manufacturer ("Teensyduino"); no port argument
  needed. It has appeared on `/dev/ttyACM0`.
- If the arrows seem dead right after launch, give it a moment to finish connecting,
  then press an arrow — there is no auto-approach in free-drive, so it should respond
  immediately once the prompt line shows the live X/Y/Z position.
