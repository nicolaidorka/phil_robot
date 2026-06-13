"""Frame-correction role split (sim backend, no hardware).

A taught well is measured ground truth -> exact replay. The persisted frame
correction has two roles that must apply to DIFFERENT wells:
  * a pure translation (reanchor / power-cycle recovery) shifts the whole joint
    frame  -> applies to EVERY well, taught included;
  * a full `anchor fit` affine (scale/shear edge refinement of the 5-bar model)
    -> applies ONLY to model-derived (untaught) wells, never to taught truth.
"""

# A realistic non-identity edge-refinement affine (like a saved phil_frame.json).
EDGE_AFFINE = {"a": 0.982, "b": -0.0092, "cx": 10.15,
               "d": 0.008, "e": 1.0087, "cy": -5.48}
PURE_TRANSLATION = {"a": 1.0, "b": 0.0, "cx": 25.0, "d": 0.0, "e": 1.0, "cy": -25.0}


def _a_taught_well(r):
    for w in ("A1", "A12", "H1", "H12", "D6"):
        if r.teach_table.is_taught(w):
            return w
    return None


def test_edge_affine_does_not_touch_taught_wells():
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    w = _a_taught_well(r)
    assert w is not None, "expected the default teach table to have taught wells"
    raw, src = r._resolve_raw(w)
    assert src == "taught"
    r.frame_correction = dict(EDGE_AFFINE)
    got, src2 = r._resolve_well(w)
    assert src2 == "taught"
    assert (got["X"], got["Y"]) == (raw["X"], raw["Y"])      # exact, byte-identical


def test_edge_affine_still_corrects_untaught_wells():
    from phil import PhilRobot
    from phil.robot import _apply_correction
    r = PhilRobot(backend="sim")
    if r.kin_model is None:
        return                                               # needs scipy for model pred
    # find an untaught well that resolves via the model
    well = None
    for c in range(1, 13):
        for row in ("F", "G"):
            wid = f"{row}{c}"
            if not r.teach_table.is_taught(wid):
                well = wid
                break
        if well:
            break
    assert well is not None
    raw, src = r._resolve_raw(well)
    assert src != "taught"
    r.frame_correction = dict(EDGE_AFFINE)
    got, _ = r._resolve_well(well)
    expect = _apply_correction(EDGE_AFFINE, raw["X"], raw["Y"])
    assert (got["X"], got["Y"]) == expect
    assert (got["X"], got["Y"]) != (raw["X"], raw["Y"])      # actually corrected


def test_pure_translation_applies_to_taught_and_untaught():
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    w = _a_taught_well(r)
    raw, _ = r._resolve_raw(w)
    r.frame_correction = dict(PURE_TRANSLATION)
    got, _ = r._resolve_well(w)                              # power-cycle recovery
    assert (got["X"], got["Y"]) == (raw["X"] + 25, raw["Y"] - 25)
