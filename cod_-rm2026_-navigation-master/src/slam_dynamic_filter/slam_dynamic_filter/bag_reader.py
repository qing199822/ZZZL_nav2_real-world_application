from __future__ import annotations

import math
import pathlib
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Mapping, Sequence

from .constants import REQUIRED_TOPIC_TYPES, TOPIC_PRIORITY


@dataclass(frozen=True)
class EvidenceRecord:
    topic: str
    source_time_ns: int
    bag_time_ns: int
    sequence: int
    message: Any
    used_bag_time_fallback: bool


@dataclass(frozen=True)
class Pose2D:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    vx: float | None = None
    vy: float | None = None
    wz: float | None = None


class BagValidationError(Exception):
    pass


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _message_stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        return _stamp_ns(stamp)
    except (AttributeError, TypeError, ValueError):
        return None


def source_timestamp_ns(topic: str, message: Any, bag_time_ns: int) -> tuple[int, bool]:
    source_time_ns = _message_stamp_ns(message)
    if source_time_ns is not None:
        return source_time_ns, False

    if topic in ("/tf", "/tf_static"):
        for transform in getattr(message, "transforms", ()):
            source_time_ns = _message_stamp_ns(transform)
            if source_time_ns is not None:
                return source_time_ns, False

    return int(bag_time_ns), True


def sort_records(records: Iterable[EvidenceRecord]) -> list[EvidenceRecord]:
    return sorted(
        records,
        key=lambda record: (
            record.source_time_ns,
            TOPIC_PRIORITY.get(record.topic, 999),
            record.sequence,
        ),
    )


def _yaw_from_quaternion(quaternion: Any) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def _optional_float(container: Any, name: str) -> float | None:
    value = getattr(container, name, None)
    return None if value is None else float(value)


def odom_to_pose2d(message: Any) -> Pose2D:
    pose = message.pose.pose
    twist = getattr(getattr(message, "twist", None), "twist", None)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    return Pose2D(
        stamp_ns=_stamp_ns(message.header.stamp),
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=_yaw_from_quaternion(pose.orientation),
        vx=_optional_float(linear, "x"),
        vy=_optional_float(linear, "y"),
        wz=_optional_float(angular, "z"),
    )


def tf_to_pose2d(
    message: Any,
    parent: str = "odom",
    child: str = "base_link",
) -> list[Pose2D]:
    parent = parent.lstrip("/")
    child = child.lstrip("/")
    poses: list[Pose2D] = []
    for transform in getattr(message, "transforms", ()):
        if (
            str(transform.header.frame_id).lstrip("/") != parent
            or str(transform.child_frame_id).lstrip("/") != child
        ):
            continue
        translation = transform.transform.translation
        poses.append(
            Pose2D(
                stamp_ns=_stamp_ns(transform.header.stamp),
                x=float(translation.x),
                y=float(translation.y),
                yaw=_yaw_from_quaternion(transform.transform.rotation),
            )
        )
    return poses


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _interpolate_optional(left: float | None, right: float | None, alpha: float) -> float | None:
    if left is None or right is None:
        return None
    return left + alpha * (right - left)


def interpolate_pose(
    records: Sequence[Pose2D],
    stamp_ns: int,
    max_gap_ns: int | None = None,
) -> Pose2D | None:
    if not records:
        return None
    ordered = sorted(records, key=lambda pose: pose.stamp_ns)
    if stamp_ns < ordered[0].stamp_ns or stamp_ns > ordered[-1].stamp_ns:
        return None
    if stamp_ns == ordered[0].stamp_ns:
        return ordered[0]

    for left, right in zip(ordered, ordered[1:]):
        if not left.stamp_ns <= stamp_ns <= right.stamp_ns:
            continue
        gap_ns = right.stamp_ns - left.stamp_ns
        if max_gap_ns is not None and gap_ns > max_gap_ns:
            return None
        if gap_ns == 0:
            return right
        alpha = (stamp_ns - left.stamp_ns) / gap_ns
        return Pose2D(
            stamp_ns=stamp_ns,
            x=left.x + alpha * (right.x - left.x),
            y=left.y + alpha * (right.y - left.y),
            yaw=_wrap_pi(left.yaw + alpha * _wrap_pi(right.yaw - left.yaw)),
            vx=_interpolate_optional(left.vx, right.vx, alpha),
            vy=_interpolate_optional(left.vy, right.vy, alpha),
            wz=_interpolate_optional(left.wz, right.wz, alpha),
        )
    return ordered[-1] if stamp_ns == ordered[-1].stamp_ns else None


def _open_rosbag_reader(bag_dir: pathlib.Path) -> Any:
    import rosbag2_py

    reader_type = getattr(rosbag2_py, "SequentialCompressionReader", None)
    if reader_type is None:
        reader_type = rosbag2_py.SequentialReader
    reader = reader_type()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def read_bag_records(
    bag_dir: pathlib.Path,
    required_topic_types: Mapping[str, str] = REQUIRED_TOPIC_TYPES,
    *,
    reader_factory: Callable[[pathlib.Path], Any] | None = None,
    message_type_resolver: Callable[[str], Any] | None = None,
    deserializer: Callable[[bytes, Any], Any] | None = None,
) -> list[EvidenceRecord]:
    if reader_factory is None:
        reader_factory = _open_rosbag_reader
    if message_type_resolver is None:
        from rosidl_runtime_py.utilities import get_message

        message_type_resolver = get_message
    if deserializer is None:
        from rclpy.serialization import deserialize_message

        deserializer = deserialize_message

    try:
        reader = reader_factory(pathlib.Path(bag_dir))
        advertised = {
            topic.name: topic.type
            for topic in reader.get_all_topics_and_types()
        }
    except Exception as exc:
        raise BagValidationError(f"unable to open or inspect bag: {exc}") from exc

    missing = sorted(set(required_topic_types) - set(advertised))
    if missing:
        raise BagValidationError(f"missing required topic: {', '.join(missing)}")

    mismatches = [
        f"{topic}: expected {expected}, got {advertised[topic]}"
        for topic, expected in required_topic_types.items()
        if advertised[topic] != expected
    ]
    if mismatches:
        raise BagValidationError(f"topic type mismatch: {'; '.join(mismatches)}")

    resolved_types: dict[str, Any] = {}
    for topic, type_name in advertised.items():
        try:
            resolved_types[topic] = message_type_resolver(type_name)
        except Exception as exc:
            raise BagValidationError(
                f"unable to resolve message type for {topic} ({type_name}): {exc}"
            ) from exc

    records: list[EvidenceRecord] = []
    sequence = 0
    while True:
        try:
            if not reader.has_next():
                break
            topic, serialized, bag_time_ns = reader.read_next()
        except Exception as exc:
            raise BagValidationError(f"corrupt bag while reading record {sequence}: {exc}") from exc

        if topic not in resolved_types:
            raise BagValidationError(f"bag record references undeclared topic: {topic}")
        try:
            message = deserializer(serialized, resolved_types[topic])
        except Exception as exc:
            raise BagValidationError(
                f"unable to deserialize {topic} record {sequence}: {exc}"
            ) from exc

        source_time_ns, used_fallback = source_timestamp_ns(topic, message, bag_time_ns)
        records.append(
            EvidenceRecord(
                topic=topic,
                source_time_ns=source_time_ns,
                bag_time_ns=int(bag_time_ns),
                sequence=sequence,
                message=message,
                used_bag_time_fallback=used_fallback,
            )
        )
        sequence += 1
    return sort_records(records)
