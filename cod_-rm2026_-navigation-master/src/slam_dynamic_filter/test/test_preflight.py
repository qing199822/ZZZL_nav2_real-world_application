import shutil
import subprocess

from slam_dynamic_filter.constants import REQUIRED_TOPIC_TYPES
from slam_dynamic_filter.preflight import check_preflight, query_topic_types


def test_query_topic_types_parses_ros2_output():
    def run(argv):
        assert argv == ["ros2", "topic", "list", "-t"]
        return subprocess.CompletedProcess(
            argv,
            0,
            stdout="/scan_raw [sensor_msgs/msg/LaserScan]\n/Odometry [nav_msgs/msg/Odometry]\n",
            stderr="",
        )

    assert query_topic_types(run)["/scan_raw"] == "sensor_msgs/msg/LaserScan"


def test_preflight_accepts_exact_topics_and_disk(tmp_path):
    def run(argv):
        lines = [f"{topic} [{topic_type}]" for topic, topic_type in REQUIRED_TOPIC_TYPES.items()]
        return subprocess.CompletedProcess(argv, 0, stdout="\n".join(lines), stderr="")

    usage = shutil._ntuple_diskusage(total=50 * 1024 ** 3, used=10, free=30 * 1024 ** 3)
    result = check_preflight(tmp_path / "not-created", run, disk_usage=lambda _: usage)
    assert result.ok
    assert result.errors == ()


def test_preflight_rejects_custommsg_and_low_disk(tmp_path):
    def run(argv):
        lines = [f"{topic} [{topic_type}]" for topic, topic_type in REQUIRED_TOPIC_TYPES.items()]
        lines[0] = "/livox/lidar [livox_ros_driver2/msg/CustomMsg]"
        return subprocess.CompletedProcess(argv, 0, stdout="\n".join(lines), stderr="")

    usage = shutil._ntuple_diskusage(total=30, used=20, free=10)
    result = check_preflight(tmp_path, run, disk_usage=lambda _: usage)
    assert not result.ok
    assert any("type mismatch /livox/lidar" in error for error in result.errors)
    assert any("free disk" in error for error in result.errors)


def test_preflight_converts_ros_query_failure_to_blocked_result(tmp_path):
    def run(argv):
        return subprocess.CompletedProcess(argv, 1, stdout="", stderr="daemon unavailable")

    usage = shutil._ntuple_diskusage(total=50, used=10, free=40 * 1024 ** 3)
    result = check_preflight(tmp_path, run, disk_usage=lambda _: usage)
    assert not result.ok
    assert "daemon unavailable" in result.errors[0]
