"""Hardware drivers for the Phil arm.

Intentionally empty: ``legacy_mc`` imports pyserial at module top, so it is left
to be imported lazily (only ``phil.robot.connect()`` with the legacy/stock backend
pulls it in). This keeps ``import phil`` and the ``sim`` backend working without pyserial.
"""
