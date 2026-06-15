#!/usr/bin/env python3
"""Probe Phil's Z height range, step-by-step, logging every position.

Run from the software/ directory:   python3 zprobe.py
You drive each move from the keyboard, so you can WATCH the arm and stop before
it hits the mechanical top. Phil is open-loop (no encoder), so the counter keeps
counting even if the motor stalls against a hard stop -- DON'T push into the top.

Commands at the prompt:
    Enter   raise Z by one step
    d       lower Z by one step
    +       double the step size
    -       halve the step size
    m <mm>  tell it the CURRENT physical height in mm (calibrates the mm readout)
    q       quit (prints + logs the max height reached)

Rough scale to start: ~8900 microsteps/mm (from your "4-5 cm for 400000" estimate).
Use `m <mm>` once at a known height to make the mm column accurate.
"""
from phil import PhilRobot

STEP0 = 50000          # ~5-6 mm per press to start
LOG = "/tmp/phil_z_height.log"
scale = 8900.0         # microsteps per mm (rough; refine with `m <mm>`)

bot = PhilRobot(backend="v2", controller_sn="16640550")
bot.connect()
z0 = bot.joint_position()["Z"]
zmax = z0
step = STEP0
print(f"\nconnected. Z start = {z0}.  +Z = UP.")
print("Enter=up  d=down  +=bigger step  -=smaller step  m <mm>=calibrate  q=quit")
print("WATCH the arm. Stop BEFORE the hard top (open-loop can't feel a stall).\n")

with open(LOG, "a") as log:
    log.write(f"--- session start, Z={z0} ---\n")
    while True:
        z = bot.joint_position()["Z"]
        c = input(f"Z={z:>8d}  (~{z/scale:6.1f} mm)  step={step} > ").strip().lower()
        if c == "q":
            break
        elif c == "d":
            bot.jog_joint(dz=-step)
        elif c == "+":
            step *= 2; continue
        elif c == "-":
            step = max(1000, step // 2); continue
        elif c.startswith("m"):
            try:
                mm = float(c.split()[1])
                if mm > 0:
                    scale = z / mm
                    print(f"  calibrated: {scale:.1f} microsteps/mm")
            except (IndexError, ValueError):
                print("  usage: m <mm>   (current physical height in mm)")
            continue
        else:
            bot.jog_joint(dz=step)
        z = bot.joint_position()["Z"]
        zmax = max(zmax, z)
        log.write(f"{z}\n"); log.flush()
        print(f"   -> Z = {z}  (~{z/scale:.1f} mm)")

print(f"\nMax Z reached this session: {zmax} microsteps  (~{zmax/scale:.1f} mm)")
print(f"Logged to {LOG}")
bot.close()
