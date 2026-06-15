"""Frame-correction role split (sim backend, no hardware).

A taught well is measured ground truth -> exact replay. The persisted frame
correction has two roles that must apply to DIFFERENT wells:
  * a pure translation (reanchor / power-cycle recovery) shifts the whole joint
    frame  -> applies to EVERY well, taught included;
  * a full `anchor fit` affine (scale/shear edge refinement) -> applies ONLY to
    model-/grid-derived (untaught) wells, never to taught truth.

Self-contained: teaches its own spread so it never depends on the live config.
"""

# A realistic non-identity edge-refinement affine (like a saved phil_frame.json).
EDGE_AFFINE = {"a": 0.982, "b": -0.0092, "cx": 10.15,
               "d": 0.008, "e": 1.0087, "cy": -5.48}
PURE_TRANSLATION = {"a": 1.0, "b": 0.0, "cx": 25.0, "d": 0.0, "e": 1.0, "cy": -25.0}

# A 2-D spread so untaught wells resolve via the grid predictor.
TAUGHT = {"A1": (600, 800), "A12": (15000, -9000), "H1": (8000, 9000),
          "H12": (16000, 200), "D6": (8000, 100)}


def _bot():
    from phil import PhilRobot
    r = PhilRobot(backend="sim")
    r.teach_table.taught = {w: {"X": x, "Y": y, "Z": 0} for w, (x, y) in TAUGHT.items()}
    return r


def test_edge_affine_does_not_touch_taught_wells():
    r = _bot()
    raw, src = r._resolve_raw("A1")
    assert src == "taught"
    r.frame_correction = dict(EDGE_AFFINE)
    got, src2 = r._resolve_well("A1")
    assert src2 == "taught"
    assert (got["X"], got["Y"]) == (raw["X"], raw["Y"])      # exact, byte-identical


def test_edge_affine_still_corrects_untaught_wells():
    from phil.robot import _apply_correction
    r = _bot()
    raw, src = r._resolve_raw("F6")                          # untaught -> grid
    assert src != "taught"
    r.frame_correction = dict(EDGE_AFFINE)
    got, _ = r._resolve_well("F6")
    expect = _apply_correction(EDGE_AFFINE, raw["X"], raw["Y"])
    assert (got["X"], got["Y"]) == expect
    assert (got["X"], got["Y"]) != (raw["X"], raw["Y"])      # actually corrected


def test_pure_translation_applies_to_taught_and_untaught():
    r = _bot()
    raw, _ = r._resolve_raw("A1")
    r.frame_correction = dict(PURE_TRANSLATION)
    got, _ = r._resolve_well("A1")                           # power-cycle recovery
    assert (got["X"], got["Y"]) == (raw["X"] + 25, raw["Y"] - 25)
