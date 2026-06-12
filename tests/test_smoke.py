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
    # subpackage entry points resolve (teach is a model, lives in geometry/)
    from phil.geometry import WellPlate as _WP, Calibration as _Cal, TeachTable as _TT  # noqa: F401


def test_no_import_cycle_jog_teach():
    # jog_teach (root entry point) imports phil.robot, which imports phil.geometry.teach.
    # If geometry/__init__ ever eagerly imported jog_teach, `import phil` would break.
    import phil  # noqa: F401
    import phil.jog_teach
    assert callable(phil.jog_teach.main)


def test_labware_loads_from_package_data():
    from phil import WellPlate
    from phil.geometry.well_plate import available_labware
    plate = WellPlate.load()                 # default plate via phil.paths
    x, y = plate.local_xy("A1")
    assert isinstance(x, float) and isinstance(y, float)
    # a merged "custom" plate still resolves by name from the single labware/ dir
    p384 = WellPlate.load("thermofisher_60180p109_384_wellplate_30ul")
    assert len(p384) > 96
    assert len(available_labware()) >= 9


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


# --- multi-corner affine frame correction --------------------------------

def test_correction_identity_is_exact_noop():
    from phil.robot import IDENTITY_CORRECTION, _is_identity, _apply_correction
    assert _is_identity(dict(IDENTITY_CORRECTION))
    for xm, ym in [(77, 388), (627, 170), (542, 249)]:
        assert _apply_correction(IDENTITY_CORRECTION, xm, ym) == (xm, ym)


def test_frame_file_backcompat_dx_dy(tmp_path):
    # An old {dx,dy} frame file loads as a pure translation.
    import json
    from phil import PhilRobot
    p = tmp_path / "phil_frame.json"
    json.dump({"dx": 4, "dy": -2, "last_x": 81, "last_y": 386}, open(p, "w"))
    r = PhilRobot(backend="sim")
    r._frame_path = str(p)
    r._load_frame()
    fc = r.frame_correction
    assert (fc["a"], fc["e"], fc["b"], fc["d"]) == (1.0, 1.0, 0.0, 0.0)
    assert (fc["cx"], fc["cy"]) == (4.0, -2.0)
    assert r.joint_offset == (4.0, -2.0)      # back-compat property


def test_fit_recovers_known_small_affine():
    from phil import PhilRobot
    from phil.robot import _apply_correction
    r = PhilRobot(backend="sim")
    if r.kin_model is None:
        return
    known = {"a": 1.001, "b": 0.0005, "cx": 3.0, "d": -0.0005, "e": 0.999, "cy": -2.0}
    r._anchor_pts = {}
    for w in r.ANCHOR_WELLS:
        m = r.kin_model.predict(w, r.plate)
        # exact (unrounded) measurements so we test the fit math, not int quantization
        meas = (known["a"] * m["X"] + known["b"] * m["Y"] + known["cx"],
                known["d"] * m["X"] + known["e"] * m["Y"] + known["cy"])
        r._anchor_pts[w] = (meas, (m["X"], m["Y"]))
    fc, info = r._fit_correction()
    assert "affine" in info
    for k in ("a", "b", "cx", "d", "e", "cy"):
        assert abs(fc[k] - known[k]) < 1e-6, (k, fc[k], known[k])


def test_fit_clamps_implausible_scale_to_translation():
    from phil import PhilRobot
    from phil.robot import _apply_correction
    r = PhilRobot(backend="sim")
    if r.kin_model is None:
        return
    big = {"a": 1.1, "b": 0.0, "cx": 2.0, "d": 0.0, "e": 0.9, "cy": -3.0}  # |a-1|=0.1 >> clamp
    r._anchor_pts = {}
    for w in r.ANCHOR_WELLS:
        m = r.kin_model.predict(w, r.plate)
        r._anchor_pts[w] = (_apply_correction(big, m["X"], m["Y"]), (m["X"], m["Y"]))
    fc, info = r._fit_correction()
    assert "translation" in info
    assert fc["a"] == 1.0 and fc["e"] == 1.0 and fc["b"] == 0.0 and fc["d"] == 0.0


def test_add_anchor_captures_uncorrected_model_prediction():
    # add_anchor must store the RAW model pred, regardless of an active correction.
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    r.connect()
    if r.kin_model is None:
        return
    r.frame_correction = {"a": 1.0, "b": 0.0, "cx": 25.0, "d": 0.0, "e": 1.0, "cy": -25.0}
    raw = r._resolve_raw("D6")[0]
    r.add_anchor("D6")
    _, model_xy = r._anchor_pts["D6"]
    assert model_xy == (raw["X"], raw["Y"])   # uncorrected, not shifted by (25,-25)
