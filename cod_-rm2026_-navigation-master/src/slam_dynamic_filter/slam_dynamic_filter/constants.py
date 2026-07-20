from __future__ import annotations

import math
import os
import pathlib
import re
from types import MappingProxyType

SCHEMA_VERSION = 1
SESSION_ID_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z-[a-z0-9][a-z0-9-]{0,47}$")
OUTPUT_ROOT_ENVLESS = "cod_mapping_sessions"
SCENARIO_ID = "static-wall-motion"

REQUIRED_TOPIC_TYPES = MappingProxyType({
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/livox/lidar_filtered": "sensor_msgs/msg/PointCloud2",
    "/livox/imu": "sensor_msgs/msg/Imu",
    "/scan_raw": "sensor_msgs/msg/LaserScan",
    "/Odometry": "nav_msgs/msg/Odometry",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
})
OPTIONAL_TOPIC_TYPES = MappingProxyType({"/scan": "sensor_msgs/msg/LaserScan"})
TOPIC_PRIORITY = MappingProxyType({
    "/tf_static": 0,
    "/tf": 10,
    "/Odometry": 20,
    "/livox/imu": 30,
    "/livox/lidar": 40,
    "/livox/lidar_filtered": 50,
    "/scan_raw": 60,
    "/scan": 70,
})

EXIT_SUCCESS = 0
EXIT_USAGE = 2
EXIT_PREFLIGHT_BLOCKED = 3
EXIT_RECORD_FAILED = 4
EXIT_ANALYSIS_INVALID = 5
EXIT_C1_BLOCKED = 6
EXIT_INTERNAL_ERROR = 7


def evidence_output_root(home: str | os.PathLike[str] | None = None) -> pathlib.Path:
    base = pathlib.Path(home) if home is not None else pathlib.Path.home()
    return base / OUTPUT_ROOT_ENVLESS


def expected_scan_beams(
    angle_min: float = -3.1416,
    angle_max: float = 3.1416,
    angle_increment: float = 0.0087,
) -> int:
    if angle_increment <= 0:
        raise ValueError("angle_increment must be positive")
    return int(math.ceil((angle_max - angle_min) / angle_increment))
