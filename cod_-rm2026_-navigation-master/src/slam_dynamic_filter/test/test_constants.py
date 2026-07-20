from slam_dynamic_filter.constants import (
    EXIT_ANALYSIS_INVALID,
    EXIT_C1_BLOCKED,
    EXIT_INTERNAL_ERROR,
    EXIT_PREFLIGHT_BLOCKED,
    EXIT_RECORD_FAILED,
    EXIT_SUCCESS,
    EXIT_USAGE,
    REQUIRED_TOPIC_TYPES,
    SCHEMA_VERSION,
    SESSION_ID_RE,
    TOPIC_PRIORITY,
    expected_scan_beams,
)


def test_schema_topics_and_exit_codes_are_locked():
    assert SCHEMA_VERSION == 1
    assert REQUIRED_TOPIC_TYPES == {
        "/livox/lidar": "sensor_msgs/msg/PointCloud2",
        "/livox/lidar_filtered": "sensor_msgs/msg/PointCloud2",
        "/livox/imu": "sensor_msgs/msg/Imu",
        "/scan_raw": "sensor_msgs/msg/LaserScan",
        "/Odometry": "nav_msgs/msg/Odometry",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
    }
    assert (
        EXIT_SUCCESS,
        EXIT_USAGE,
        EXIT_PREFLIGHT_BLOCKED,
        EXIT_RECORD_FAILED,
        EXIT_ANALYSIS_INVALID,
        EXIT_C1_BLOCKED,
        EXIT_INTERNAL_ERROR,
    ) == (0, 2, 3, 4, 5, 6, 7)


def test_session_id_regex_and_topic_priority_are_stable():
    assert SESSION_ID_RE.fullmatch("20260720T010203Z-static-wall-motion")
    assert not SESSION_ID_RE.fullmatch("../escape")
    assert TOPIC_PRIORITY["/tf_static"] < TOPIC_PRIORITY["/tf"] < TOPIC_PRIORITY["/scan_raw"]


def test_converter_beam_count_characterization_is_723():
    assert expected_scan_beams() == 723
