import math
import sys
from types import SimpleNamespace

import pytest

from slam_dynamic_filter.bag_reader import (
    _open_rosbag_reader,
    BagValidationError,
    EvidenceRecord,
    Pose2D,
    interpolate_pose,
    odom_to_pose2d,
    read_bag_records,
    sort_records,
    source_timestamp_ns,
    tf_to_pose2d,
)


def test_default_reader_supports_file_compressed_bags(monkeypatch, tmp_path):
    opened = []

    class CompressionReader:
        def open(self, storage_options, converter_options):
            opened.append((storage_options, converter_options))

    fake_rosbag2 = SimpleNamespace(
        SequentialCompressionReader=CompressionReader,
        SequentialReader=lambda: pytest.fail("plain reader used for compressed bag"),
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda *args: args,
    )
    monkeypatch.setitem(sys.modules, "rosbag2_py", fake_rosbag2)

    reader = _open_rosbag_reader(tmp_path)

    assert isinstance(reader, CompressionReader)
    assert opened[0][0] == {"uri": str(tmp_path), "storage_id": "sqlite3"}


def stamp(stamp_ns):
    return SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


def quaternion(yaw):
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def test_sort_records_uses_source_time_priority_sequence_and_preserves_duplicates():
    records = [
        EvidenceRecord("/scan_raw", 100, 500, 2, "scan-a", False),
        EvidenceRecord("/tf", 100, 400, 0, "tf", False),
        EvidenceRecord("/scan_raw", 100, 300, 1, "scan-b", False),
    ]
    ordered = sort_records(records)
    assert [record.message for record in ordered] == ["tf", "scan-b", "scan-a"]


def test_source_timestamp_uses_header_tf_and_bag_fallback():
    header_message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(123)))
    assert source_timestamp_ns("/scan_raw", header_message, 999) == (123, False)
    transform = SimpleNamespace(header=SimpleNamespace(stamp=stamp(456)))
    tf_message = SimpleNamespace(transforms=[transform])
    assert source_timestamp_ns("/tf", tf_message, 999) == (456, False)
    assert source_timestamp_ns("/livox/lidar", SimpleNamespace(), 999) == (999, True)


def test_pose_normalization_and_shortest_yaw_interpolation():
    odom_message = SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(5)),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0),
            orientation=quaternion(math.radians(170)),
        )),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.2),
            angular=SimpleNamespace(z=0.3),
        )),
    )
    pose = odom_to_pose2d(odom_message)
    assert (pose.x, pose.y, pose.vx, pose.vy, pose.wz) == (1.0, 2.0, 0.1, 0.2, 0.3)
    poses = [
        Pose2D(0, 0.0, 0.0, math.radians(170)),
        Pose2D(10, 10.0, 0.0, math.radians(-170)),
    ]
    mid = interpolate_pose(poses, 5)
    assert mid.x == 5.0
    assert abs(abs(mid.yaw) - math.pi) < 1e-9


def test_tf_selection_and_missing_pose_gap():
    matching = SimpleNamespace(
        header=SimpleNamespace(frame_id="odom", stamp=stamp(10)),
        child_frame_id="base_link",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0),
            rotation=quaternion(0.2),
        ),
    )
    ignored = SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=stamp(10)),
        child_frame_id="odom",
        transform=matching.transform,
    )
    poses = tf_to_pose2d(SimpleNamespace(transforms=[ignored, matching]))
    assert len(poses) == 1 and poses[0].x == 1.0
    sparse = [Pose2D(0, 0.0, 0.0, 0.0), Pose2D(100, 1.0, 0.0, 0.0)]
    assert interpolate_pose(sparse, 50, max_gap_ns=20) is None


class FakeReader:
    def __init__(self, topic_types, records):
        self.topic_types = topic_types
        self.records = list(records)
        self.index = 0

    def get_all_topics_and_types(self):
        return [
            SimpleNamespace(name=name, type=type_name)
            for name, type_name in self.topic_types.items()
        ]

    def has_next(self):
        return self.index < len(self.records)

    def read_next(self):
        record = self.records[self.index]
        self.index += 1
        return record


def test_read_bag_validates_types_before_deserializing(tmp_path):
    required = {"/scan_raw": "sensor_msgs/msg/LaserScan", "/tf": "tf2_msgs/msg/TFMessage"}
    reader = FakeReader(required, [("/tf", b"tf", 100), ("/scan_raw", b"scan", 100)])
    messages = {
        b"tf": SimpleNamespace(transforms=[
            SimpleNamespace(header=SimpleNamespace(stamp=stamp(90)))
        ]),
        b"scan": SimpleNamespace(header=SimpleNamespace(stamp=stamp(95))),
    }
    records = read_bag_records(
        tmp_path,
        required,
        reader_factory=lambda _: reader,
        message_type_resolver=lambda type_name: type_name,
        deserializer=lambda data, _: messages[data],
    )
    assert [record.topic for record in records] == ["/tf", "/scan_raw"]
    assert all(not record.used_bag_time_fallback for record in records)

    bad_reader = FakeReader({"/scan_raw": "livox_ros_driver2/msg/CustomMsg"}, [])
    with pytest.raises(BagValidationError, match="type mismatch"):
        read_bag_records(
            tmp_path,
            {"/scan_raw": "sensor_msgs/msg/LaserScan"},
            reader_factory=lambda _: bad_reader,
            message_type_resolver=lambda value: value,
            deserializer=lambda data, value: data,
        )


def test_read_bag_rejects_missing_topic_and_deserialization_failure(tmp_path):
    with pytest.raises(BagValidationError, match="missing required topic"):
        read_bag_records(
            tmp_path,
            {"/scan_raw": "sensor_msgs/msg/LaserScan"},
            reader_factory=lambda _: FakeReader({}, []),
            message_type_resolver=lambda value: value,
            deserializer=lambda data, value: data,
        )
    reader = FakeReader(
        {"/scan_raw": "sensor_msgs/msg/LaserScan"},
        [("/scan_raw", b"bad", 1)],
    )
    with pytest.raises(BagValidationError, match="deserialize"):
        read_bag_records(
            tmp_path,
            {"/scan_raw": "sensor_msgs/msg/LaserScan"},
            reader_factory=lambda _: reader,
            message_type_resolver=lambda value: value,
            deserializer=lambda data, value: (_ for _ in ()).throw(ValueError("bad")),
        )
