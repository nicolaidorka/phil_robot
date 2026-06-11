"""Single source of truth for the package's data-file locations.

Every module imports its config/labware paths from here instead of computing
``os.path.dirname(__file__)`` itself, so modules can live in subpackages
(``geometry/`` ...) while the data dirs stay at the package root.
"""
import os

_PKG = os.path.dirname(os.path.abspath(__file__))   # always the phil/ root

CONFIG_DIR = os.path.join(_PKG, "config")
LABWARE_DIRS = [os.path.join(_PKG, "labware")]      # all plate JSON in one place
DEFAULT_LABWARE = os.path.join(_PKG, "labware", "eppendorf_twintec_lobind_96_pcr.json")

DEFAULT_TEACH_PATH = os.path.join(CONFIG_DIR, "phil_teach.json")
DEFAULT_CALIBRATION = os.path.join(CONFIG_DIR, "phil_calibration.json")
DEFAULT_KIN_PATH = os.path.join(CONFIG_DIR, "phil_kinematics.json")
FRAME_FILENAME = "phil_frame.json"
