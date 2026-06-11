"""Smoke tests for the phil package — sim backend only, no hardware required.

These guard the things the subpackage refactor could break: the public API,
the import-cycle hazard, the data-path resolution (config/labware), and the
sim prediction values.
"""


def test_public_api_imports():
    import phil
    from phil import (
        PhilRobot,
        SimulatedBackend,
        PhilHandshakeError,
        WellPlate,
        Well,
        Calibration,
        ReferencePoint,
        TeachTable,
    )
    assert phil.constants is not None
    # subpackage entry points resolve
    from phil.geometry import WellPlate as _WP, Calibration as _Cal  # noqa: F401
    from phil.teaching import TeachTable as _TT  # noqa: F401


def test_no_import_cycle_jog_teach():
    # jog_teach imports phil.robot, which imports phil.teaching.teach.
    # If teaching/__init__ ever eagerly imports jog_teach, `import phil` breaks.
    import phil  # noqa: F401
    from phil.teaching import jog_teach
    assert callable(jog_teach.main)


def test_labware_loads_from_package_data():
    from phil import WellPlate
    plate = WellPlate.load()                 # default plate via phil.paths
    x, y = plate.local_xy("A1")
    assert isinstance(x, float) and isinstance(y, float)


def test_sim_predicts_taught_and_untaught_wells():
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    r.connect()
    for well in ("A1", "B10", "G9"):
        p = r.predict_well(well)
        assert set(p) == {"X", "Y", "Z"}
        assert all(isinstance(v, int) for v in p.values())
    # frame state file resolves into config/ (not orphaned by the move)
    assert r._frame_path.replace("\\", "/").endswith("config/phil_frame.json")


def test_kinematics_state_survived_the_move():
    # The fitted 5-bar model must still load from config/phil_kinematics.json.
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    if r.kin_model is not None:               # scipy present
        assert r.kin_model.is_fitted
        assert r.kin_model.rms_mm < 1.0       # the fitted model is ~0.21 mm
