import math
import os

import pytest

from ur3e_sysid import inertia

_HAVE_YAML = os.path.isfile(inertia.DEFAULT_PHYSICAL_PARAMS) and os.path.isfile(inertia.DEFAULT_KINEMATICS)
pytestmark = pytest.mark.skipif(not _HAVE_YAML, reason="ur_description UR3e config not installed")

ID_POSE = (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)


def test_effective_inertia_positive_and_hierarchy():
    I = inertia.effective_inertia(ID_POSE)
    assert set(I) == set(inertia.JOINT_NAMES)
    for name, val in I.items():
        assert math.isfinite(val) and val > 0.0, (name, val)
    # proximal joints carry more outboard mass than distal ones.
    assert I["shoulder_lift_joint"] > I["elbow_joint"] > I["wrist_1_joint"]
    assert I["wrist_1_joint"] > I["wrist_3_joint"]
    assert I["shoulder_lift_joint"] == max(I.values())
    # wrist_3 only moves its own small link -> sub-1e-2 kg m^2.
    assert I["wrist_3_joint"] < 1e-2


def test_pose_dependence():
    straight = inertia.effective_inertia((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    folded = inertia.effective_inertia(ID_POSE)
    # shoulder_pan inertia about the vertical axis changes when the arm folds.
    assert not math.isclose(straight["shoulder_pan_joint"], folded["shoulder_pan_joint"], rel_tol=1e-3)
