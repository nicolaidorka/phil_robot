"""Make `import phil` work when running the tests in-place (without `pip install`).

Adds the `software/` dir (which contains the `phil` package) to sys.path.
After `pip install -e .` this is harmless / redundant.
"""
import os
import sys

_SOFTWARE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "software")
if _SOFTWARE not in sys.path:
    sys.path.insert(0, _SOFTWARE)
