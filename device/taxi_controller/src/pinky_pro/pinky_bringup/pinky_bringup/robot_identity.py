from __future__ import annotations

from pathlib import Path

import yaml

DEFAULT_PINKY_ID = 0
DEFAULT_MAP_FRAME = "map"


def normalize_pinky_id(value: int | str | None) -> int:
    return int(value if value is not None else DEFAULT_PINKY_ID) & 0xFF


def robot_namespace(pinky_id: int | str | None) -> str:
    return f"pinky_{normalize_pinky_id(pinky_id)}"


def robot_frame(pinky_id: int | str | None, frame_name: str) -> str:
    return f"{robot_namespace(pinky_id)}/{frame_name}"


def build_robot_identity(pinky_id: int | str | None) -> dict[str, str | int]:
    pinky_id = normalize_pinky_id(pinky_id)
    namespace = robot_namespace(pinky_id)
    return {
        "pinky_id": pinky_id,
        "namespace": namespace,
        "map_frame_id": DEFAULT_MAP_FRAME,
        "odom_frame_id": robot_frame(pinky_id, "odom"),
        "base_frame_id": robot_frame(pinky_id, "base_footprint"),
        "lidar_frame_id": robot_frame(pinky_id, "rplidar_link"),
    }


def load_pinky_id_from_params_file(params_file: str | Path) -> int:
    path = Path(params_file)
    if not path.exists():
        return DEFAULT_PINKY_ID

    data = yaml.safe_load(path.read_text()) or {}
    for node_name in ("pinky_bringup", "gps_odometry_calibrator", "gazebo_gps_publisher"):
        params = (data.get(node_name) or {}).get("ros__parameters") or {}
        if "pinky_id" in params:
            return normalize_pinky_id(params["pinky_id"])
    return DEFAULT_PINKY_ID
