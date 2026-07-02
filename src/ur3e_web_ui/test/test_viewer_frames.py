from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from ur3e_web_ui.urdf_provider import _generate_with_xacro


VIEWER_SOURCE = Path(__file__).parents[1] / "ur3e_web_ui" / "static" / "js" / "viewer3d.js"


def _joint(root: ET.Element, name: str) -> ET.Element:
    joint = root.find(f".//joint[@name='{name}']")
    assert joint is not None, f"missing joint {name}"
    return joint


def _link_name(joint: ET.Element, tag: str) -> str:
    element = joint.find(tag)
    assert element is not None, f"{joint.attrib.get('name')} missing {tag}"
    return str(element.attrib["link"])


def _rpy(joint: ET.Element) -> tuple[float, float, float]:
    origin = joint.find("origin")
    assert origin is not None, f"{joint.attrib.get('name')} missing origin"
    return tuple(float(value) for value in origin.attrib.get("rpy", "0 0 0").split())


def test_urdf_base_link_is_the_viewer_and_isaac_root_frame():
    pytest.importorskip("xacro")

    root = ET.fromstring(_generate_with_xacro())

    world_to_base_link = _joint(root, "base_joint")
    assert _link_name(world_to_base_link, "parent") == "world"
    assert _link_name(world_to_base_link, "child") == "base_link"
    assert _rpy(world_to_base_link) == pytest.approx((0.0, 0.0, 0.0))

    base_link_to_base = _joint(root, "base_link-base_fixed_joint")
    assert _link_name(base_link_to_base, "parent") == "base_link"
    assert _link_name(base_link_to_base, "child") == "base"
    assert _rpy(base_link_to_base) == pytest.approx((0.0, 0.0, math.pi))

    base_link_to_inertia = _joint(root, "base_link-base_link_inertia")
    assert _link_name(base_link_to_inertia, "parent") == "base_link"
    assert _link_name(base_link_to_inertia, "child") == "base_link_inertia"
    assert _rpy(base_link_to_inertia) == pytest.approx((0.0, 0.0, math.pi))


def test_viewer_keeps_robot_root_in_base_link_not_ur_base():
    source = VIEWER_SOURCE.read_text(encoding="utf-8")

    assert "URDF root is `base_link`" in source
    assert "root.quaternion.copy(ROS_TO_THREE_Q);" in source
    assert "would turn the displayed robot 180 deg away from Isaac" in source
    assert "this.orientRobotRoot(this.robot);" in source
    assert "this.orientRobotRoot(this.ghost);" in source
    assert "this.orientRobotRoot(this.replayGhost);" in source
    assert "this.orientRobotRoot(this.policyGhost);" in source


def test_viewer_attaches_isaac_hoop_visual_to_wrist_3_link():
    source = VIEWER_SOURCE.read_text(encoding="utf-8")

    assert 'const ISAAC_HOOP_LINK = "wrist_3_link";' in source
    assert "const ISAAC_HOOP_CENTER_M = [-0.5, 0.0, 0.0];" in source
    assert "const ISAAC_HOOP_NORMAL = [0.0, 0.0, -1.0];" in source
    assert "const ISAAC_HOOP_VISUAL_RADIUS_M = 0.15;" in source
    assert "const ISAAC_HOOP_VALIDATION_RADIUS_M = 0.05;" in source
    assert 'this.attachIsaacHoop(this.robot, "robot");' in source
    assert 'this.attachIsaacHoop(this.ghost, "ghost");' in source
    assert 'this.attachIsaacHoop(this.replayGhost, "ghost");' in source
    assert 'this.attachIsaacHoop(this.policyGhost, "ghost");' in source
    assert "new THREE.TorusGeometry(ISAAC_HOOP_VISUAL_RADIUS_M" in source
    assert "new THREE.CylinderGeometry(ISAAC_HOOP_ROD_RADIUS_M" in source


def test_viewer_separates_base_and_base_link_coordinate_conversions():
    source = VIEWER_SOURCE.read_text(encoding="utf-8")

    assert "base (x,y,z) -> three.js (-x, z, y)" in source
    assert "return new THREE.Vector3(-x, z, y);" in source
    assert "base_link (x,y,z) -> three.js (x, z, -y)" in source
    assert "return new THREE.Vector3(x, z, -y);" in source
    assert "p0: [position.x, -position.z, position.y]," in source
