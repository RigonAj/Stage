"""Make ``ur3e_live_catch`` importable in-tree (no colcon install needed).

The pure-logic modules use only the stdlib, so these tests run anywhere a plain
``python3`` is available — including this cloud environment with no ROS/torch.
"""

import sys
from pathlib import Path

# .../src/ur3e_live_catch  (the dir that *contains* the ur3e_live_catch package)
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def repo_root() -> Path:
    """Walk up until we find the repository data directory."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "data" / "ur3e_rollouts").is_dir():
            return parent
    raise RuntimeError("could not locate repo root (data/ur3e_rollouts not found)")
