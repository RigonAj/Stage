from __future__ import annotations

from pathlib import Path
import threading


def _default_ur_description_share() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("ur_description"))
    except Exception:
        for distro in ("jazzy", "humble"):
            path = Path(f"/opt/ros/{distro}/share/ur_description")
            if path.is_dir():
                return path
        return Path("/opt/ros/jazzy/share/ur_description")


UR_DESCRIPTION_SHARE = str(_default_ur_description_share())
UR_XACRO_PATH = str(Path(UR_DESCRIPTION_SHARE) / "urdf" / "ur.urdf.xacro")


class UrdfCache:
    """Holds the robot URDF.

    Preferred source is the /robot_description topic published by the running
    driver (it includes the extracted kinematics calibration). When the driver
    is not running, falls back to generating a default-kinematics URDF with
    xacro so the 3D viewer still works.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._topic_urdf: str | None = None
        self._xacro_urdf: str | None = None

    def set_from_topic(self, urdf_xml: str) -> None:
        with self._lock:
            self._topic_urdf = urdf_xml

    def get(self) -> tuple[str, str]:
        """Returns (urdf_xml, source) where source is "topic" or "xacro"."""
        with self._lock:
            if self._topic_urdf is not None:
                return self._topic_urdf, "topic"
            if self._xacro_urdf is not None:
                return self._xacro_urdf, "xacro"

        generated = _generate_with_xacro()
        with self._lock:
            self._xacro_urdf = generated
            if self._topic_urdf is not None:
                return self._topic_urdf, "topic"
            return self._xacro_urdf, "xacro"


def _generate_with_xacro() -> str:
    import xacro

    document = xacro.process_file(UR_XACRO_PATH, mappings={"ur_type": "ur3e", "name": "ur"})
    return document.toxml()
