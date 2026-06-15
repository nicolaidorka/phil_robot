"""Coordinated final-approach tests (sim backend, no hardware).

Guard the two motion properties the fix relies on:
  * `_approach_joints` advances the two arms TOGETHER (alternating +X/+Y relative
    steps), not X-all-then-Y, and lands exactly on the target count;
  * `_confirm_joints` waits for the reported counts to actually reach target and
    times out cleanly when they can't (the legacy firmware only ACKs the last cmd).

These rely on the sim relative-jog sign matching the firmware/teach convention (a
`+` jog raises the reported count) so the absolute pre-position + relative creep
converge — see SimulatedBackend.move_*_usteps.
"""


def _spy_absolute_moves(r):
    """Record (x, y) targets for every ABSOLUTE move while still applying it."""
    calls = []
    box, boy = r.mc.move_x_to_usteps, r.mc.move_y_to_usteps
    state = {"x": None, "y": None}
    def mx(u):
        state["x"] = u; box(u)
    def my(u):
        state["y"] = u; boy(u); calls.append((state["x"], state["y"]))
    r.mc.move_x_to_usteps = mx
    r.mc.move_y_to_usteps = my
    return calls


def test_approach_is_two_step_from_below_and_lands_on_target():
    # On this firmware the relative MOVE jog over-travels but absolute MOVETO is
    # accurate, so the approach pre-positions at (x-pre, y-pre) then moves straight
    # onto (x, y) -- the final leg is +pre on each axis, arriving +X,+Y. Big legs
    # are chunked (anti step-loss), so we check the waypoints appear in order.
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    r.connect()
    r.teach_table.taught = {}                        # <4 wells -> no joint-box clamp;
    r._move_joints_to(x=0, y=0)                      # test pure approach, not clamping
    calls = _spy_absolute_moves(r)

    r._approach_joints(100, 200)

    pre = r.APPROACH_PRE_USTEPS
    assert (100 - pre, 200 - pre) in calls           # pre-position waypoint hit
    assert calls[-1] == (100, 200)                   # final leg lands on target
    # every leg approaches from below (no command exceeds the target) -> +X,+Y
    assert all(cx <= 100 and cy <= 200 for cx, cy in calls)

    j = r.joint_position()
    assert (j["X"], j["Y"]) == (100, 200)            # exact arrival


def test_approach_handles_negative_targets():
    # The frame can sit at negative counts; the approach must reach them (no clamp).
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    r.connect()
    r.teach_table.taught = {}                        # disable the joint-box clamp
    r._move_joints_to(x=0, y=0)
    r._approach_joints(-52, -168)
    j = r.joint_position()
    assert (j["X"], j["Y"]) == (-52, -168)


def test_confirm_joints_true_when_arrived_false_on_timeout():
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    r.connect()
    r.teach_table.taught = {}                        # disable the joint-box clamp
    r._move_joints_to(x=120, y=140)
    assert r._confirm_joints(x=120, y=140, tol=0, timeout=0.5) is True
    # an unreachable target (sim never moves on its own) must time out, not hang
    assert r._confirm_joints(x=999999, y=140, tol=0, timeout=0.2) is False
