"""Make ``ur3e_sysid`` importable when running pytest directly from the source tree
(without colcon build/install). The math modules need only numpy/scipy (ROS python)."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
