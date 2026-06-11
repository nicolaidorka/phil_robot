"""Backwards-compatible shim so ``python -m phil.jog_teach`` keeps working.

The implementation now lives in :mod:`phil.teaching.jog_teach`.
"""
from .teaching.jog_teach import main

if __name__ == "__main__":
    main()
