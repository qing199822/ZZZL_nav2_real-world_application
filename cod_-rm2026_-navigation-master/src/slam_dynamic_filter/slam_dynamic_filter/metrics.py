from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.spatial import cKDTree

from .bag_reader import EvidenceRecord, Pose2D, interpolate_pose

STATIONARY_LINEAR_SPEED_MAX_MPS = 0.03
STATIONARY_ANGULAR_SPEED_MAX_RADPS = 0.03
STATIONARY_POSE_GAP_MAX_NS = 200_000_000
TF_PAIRING_TOLERANCE_NS = 50_000_000
SCAN_PAIR_GAP_MAX_NS = 200_000_000
OVERLAP_RESIDUAL_MAX_M = 0.20


@dataclass(frozen=True)
class ScanMetrics:
    duration_sec: float
    rate_hz: float
    rate_cv: float
    beam_count: int
    stationary_stable_bins: int
    stationary_range_mad_p95_m: float | None


@dataclass(frozen=True)
class OdomTfMetrics:
    tf_coverage: float
    position_p95_m: float | None
    yaw_p95_rad: float | None
    translation_jump_max_m: float
    yaw_jump_max_rad: float


@dataclass(frozen=True)
class CompensatedResidualMetrics:
    nn_residual_p95_m: float | None
    overlap_median: float | None
    compared_pairs: int


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _scan_stamp_ns(scan: Any) -> int:
    stamp = scan.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _finite_twist(pose: Pose2D) -> bool:
    return (
        pose.vx is not None
        and pose.vy is not None
        and pose.wz is not None
        and math.isfinite(pose.vx)
        and math.isfinite(pose.vy)
        and math.isfinite(pose.wz)
    )


def _pose_delta_stationary(ordered: Sequence[Pose2D], stamp_ns: int) -> bool:
    if len(ordered) < 2:
        return False
    stamps = [pose.stamp_ns for pose in ordered]
    index = bisect.bisect_left(stamps, stamp_ns)
    if index <= 0:
        left, right = ordered[0], ordered[1]
    elif index >= len(ordered):
        left, right = ordered[-2], ordered[-1]
    else:
        left, right = ordered[index - 1], ordered[index]
    delta_ns = right.stamp_ns - left.stamp_ns
    if delta_ns <= 0 or delta_ns > STATIONARY_POSE_GAP_MAX_NS:
        return False
    delta_sec = delta_ns / 1_000_000_000
    linear_speed = math.hypot(right.x - left.x, right.y - left.y) / delta_sec
    angular_speed = abs(_wrap_pi(right.yaw - left.yaw)) / delta_sec
    return (
        linear_speed <= STATIONARY_LINEAR_SPEED_MAX_MPS
        and angular_speed <= STATIONARY_ANGULAR_SPEED_MAX_RADPS
    )


def _stationary_mask(odom: Sequence[Pose2D], stamps: Sequence[int]) -> list[bool]:
    ordered = sorted(odom, key=lambda pose: pose.stamp_ns)
    stationary: list[bool] = []
    for stamp_ns in stamps:
        pose = interpolate_pose(ordered, stamp_ns, max_gap_ns=STATIONARY_POSE_GAP_MAX_NS)
        if pose is None:
            stationary.append(False)
        elif _finite_twist(pose):
            pose_delta_stationary = _pose_delta_stationary(ordered, stamp_ns)
            stationary.append(
                math.hypot(pose.vx, pose.vy) <= STATIONARY_LINEAR_SPEED_MAX_MPS
                and abs(pose.wz) <= STATIONARY_ANGULAR_SPEED_MAX_RADPS
                and pose_delta_stationary
            )
        else:
            stationary.append(_pose_delta_stationary(ordered, stamp_ns))
    return stationary


def compute_scan_metrics(
    scan_records: Sequence[Any],
    odom_poses: Sequence[Pose2D],
) -> ScanMetrics:
    if not scan_records:
        return ScanMetrics(0.0, 0.0, 0.0, 0, 0, None)

    stamps = [_scan_stamp_ns(scan) for scan in scan_records]
    ordered_stamps = sorted(stamps)
    duration_sec = (
        (ordered_stamps[-1] - ordered_stamps[0]) / 1_000_000_000
        if len(ordered_stamps) > 1
        else 0.0
    )
    intervals = (
        np.diff(ordered_stamps).astype(float) / 1_000_000_000
        if len(ordered_stamps) > 1
        else np.asarray([], dtype=float)
    )
    positive_intervals = intervals[intervals > 0.0]
    median_interval = float(np.median(positive_intervals)) if positive_intervals.size else 0.0
    rate_hz = 1.0 / median_interval if median_interval > 0.0 else 0.0
    mean_interval = float(np.mean(positive_intervals)) if positive_intervals.size else 0.0
    rate_cv = (
        float(np.std(positive_intervals) / mean_interval)
        if mean_interval > 0.0
        else 0.0
    )

    beam_counts = {len(scan.ranges) for scan in scan_records}
    beam_count = next(iter(beam_counts)) if len(beam_counts) == 1 else -1
    if beam_count <= 0:
        return ScanMetrics(duration_sec, rate_hz, rate_cv, beam_count, 0, None)

    stationary_scans = [
        scan
        for scan, stationary in zip(
            scan_records,
            _stationary_mask(odom_poses, stamps),
        )
        if stationary
    ]
    if len(stationary_scans) < 2:
        return ScanMetrics(duration_sec, rate_hz, rate_cv, beam_count, 0, None)

    ranges = np.asarray(
        [
            [float(value) if math.isfinite(value) else np.nan for value in scan.ranges]
            for scan in stationary_scans
        ],
        dtype=float,
    )
    finite_ratio = np.mean(np.isfinite(ranges), axis=0)
    qualified = finite_ratio >= 0.8
    stable_bins = int(np.sum(qualified))
    mad_p95_m: float | None = None
    if stable_bins:
        qualified_ranges = ranges[:, qualified]
        medians = np.nanmedian(qualified_ranges, axis=0)
        mad = np.nanmedian(np.abs(qualified_ranges - medians), axis=0)
        mad_p95_m = float(np.nanpercentile(mad, 95))
    return ScanMetrics(
        duration_sec,
        float(rate_hz),
        rate_cv,
        beam_count,
        stable_bins,
        mad_p95_m,
    )


def _pose_near_stamp(
    poses: Sequence[Pose2D],
    stamp_ns: int,
    tolerance_ns: int,
) -> Pose2D | None:
    if not poses:
        return None
    ordered = sorted(poses, key=lambda pose: pose.stamp_ns)
    stamps = [pose.stamp_ns for pose in ordered]
    index = bisect.bisect_left(stamps, stamp_ns)
    if index < len(ordered) and ordered[index].stamp_ns == stamp_ns:
        return ordered[index]

    if 0 < index < len(ordered):
        left = ordered[index - 1]
        right = ordered[index]
        if (
            stamp_ns - left.stamp_ns <= tolerance_ns
            and right.stamp_ns - stamp_ns <= tolerance_ns
        ):
            return interpolate_pose((left, right), stamp_ns)

    candidates = []
    if index > 0:
        candidates.append(ordered[index - 1])
    if index < len(ordered):
        candidates.append(ordered[index])
    nearest = min(candidates, key=lambda pose: abs(pose.stamp_ns - stamp_ns))
    return nearest if abs(nearest.stamp_ns - stamp_ns) <= tolerance_ns else None


def compute_odom_tf_metrics(
    odom: Sequence[Pose2D],
    tf: Sequence[Pose2D],
) -> OdomTfMetrics:
    ordered_odom = sorted(odom, key=lambda pose: pose.stamp_ns)
    position_errors: list[float] = []
    yaw_errors: list[float] = []
    for odom_pose in ordered_odom:
        tf_pose = _pose_near_stamp(tf, odom_pose.stamp_ns, TF_PAIRING_TOLERANCE_NS)
        if tf_pose is None:
            continue
        position_errors.append(math.hypot(odom_pose.x - tf_pose.x, odom_pose.y - tf_pose.y))
        yaw_errors.append(abs(_wrap_pi(odom_pose.yaw - tf_pose.yaw)))

    translation_jumps = [
        math.hypot(right.x - left.x, right.y - left.y)
        for left, right in zip(ordered_odom, ordered_odom[1:])
    ]
    yaw_jumps = [
        abs(_wrap_pi(right.yaw - left.yaw))
        for left, right in zip(ordered_odom, ordered_odom[1:])
    ]
    return OdomTfMetrics(
        tf_coverage=len(position_errors) / len(ordered_odom) if ordered_odom else 0.0,
        position_p95_m=(
            float(np.percentile(position_errors, 95)) if position_errors else None
        ),
        yaw_p95_rad=float(np.percentile(yaw_errors, 95)) if yaw_errors else None,
        translation_jump_max_m=max(translation_jumps, default=0.0),
        yaw_jump_max_rad=max(yaw_jumps, default=0.0),
    )


def _endpoints(scan: Any, pose: Pose2D) -> np.ndarray:
    points: list[tuple[float, float]] = []
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    for index, range_m in enumerate(scan.ranges):
        if (
            not math.isfinite(range_m)
            or range_m < scan.range_min
            or range_m > scan.range_max
        ):
            continue
        angle = scan.angle_min + index * scan.angle_increment
        base_x = range_m * math.cos(angle)
        base_y = range_m * math.sin(angle)
        points.append((
            pose.x + cos_yaw * base_x - sin_yaw * base_y,
            pose.y + sin_yaw * base_x + cos_yaw * base_y,
        ))
    return np.asarray(points, dtype=float).reshape((-1, 2))


def compute_compensated_residuals(
    scans: Sequence[Any],
    odom: Sequence[Pose2D],
) -> CompensatedResidualMetrics:
    ordered_scans = sorted(scans, key=_scan_stamp_ns)
    residuals: list[float] = []
    overlaps: list[float] = []
    compared_pairs = 0
    for left_scan, right_scan in zip(ordered_scans, ordered_scans[1:]):
        left_stamp = _scan_stamp_ns(left_scan)
        right_stamp = _scan_stamp_ns(right_scan)
        gap_ns = right_stamp - left_stamp
        if gap_ns < 0 or gap_ns > SCAN_PAIR_GAP_MAX_NS:
            continue
        left_pose = interpolate_pose(odom, left_stamp, max_gap_ns=SCAN_PAIR_GAP_MAX_NS)
        right_pose = interpolate_pose(odom, right_stamp, max_gap_ns=SCAN_PAIR_GAP_MAX_NS)
        if left_pose is None or right_pose is None:
            continue
        left_points = _endpoints(left_scan, left_pose)
        right_points = _endpoints(right_scan, right_pose)
        if not len(left_points) or not len(right_points):
            continue
        distances, _ = cKDTree(left_points).query(right_points, k=1)
        residuals.extend(float(distance) for distance in distances)
        overlaps.append(float(np.mean(distances <= OVERLAP_RESIDUAL_MAX_M)))
        compared_pairs += 1
    return CompensatedResidualMetrics(
        nn_residual_p95_m=(
            float(np.percentile(residuals, 95)) if residuals else None
        ),
        overlap_median=float(np.median(overlaps)) if overlaps else None,
        compared_pairs=compared_pairs,
    )


def count_timestamp_reversals(
    records: Sequence[EvidenceRecord],
) -> dict[str, int]:
    last_source_time: dict[tuple[str, str, str], int] = {}
    counts: dict[str, int] = {}
    for record in sorted(records, key=lambda item: item.sequence):
        counts.setdefault(record.topic, 0)
        streams = []
        if record.topic in ("/tf", "/tf_static"):
            for transform in getattr(record.message, "transforms", ()):
                try:
                    stamp = transform.header.stamp
                    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    parent = str(transform.header.frame_id).lstrip("/")
                    child = str(transform.child_frame_id).lstrip("/")
                    streams.append(((record.topic, parent, child), stamp_ns))
                except (AttributeError, TypeError, ValueError):
                    continue
        if not streams:
            streams.append(((record.topic, "", ""), record.source_time_ns))
        for stream, source_time_ns in streams:
            previous = last_source_time.get(stream)
            if previous is not None and source_time_ns < previous:
                counts[record.topic] += 1
            last_source_time[stream] = source_time_ns
    return counts


def compute_profile_metrics(
    samples: Sequence[Mapping[str, Any]],
) -> dict[str, float | int | bool | None]:
    required = (
        "total_cpu_percent",
        "available_memory_bytes",
        "disk_free_bytes",
    )
    complete = bool(samples) and all(
        all(
            key in sample
            and isinstance(sample[key], (int, float))
            and math.isfinite(float(sample[key]))
            for key in required
        )
        for sample in samples
    )
    cpu = [
        float(sample["total_cpu_percent"])
        for sample in samples
        if "total_cpu_percent" in sample
        and isinstance(sample["total_cpu_percent"], (int, float))
        and math.isfinite(float(sample["total_cpu_percent"]))
    ]
    memory = [
        float(sample["available_memory_bytes"])
        for sample in samples
        if "available_memory_bytes" in sample
        and isinstance(sample["available_memory_bytes"], (int, float))
        and math.isfinite(float(sample["available_memory_bytes"]))
    ]
    disk = [
        float(sample["disk_free_bytes"])
        for sample in samples
        if "disk_free_bytes" in sample
        and isinstance(sample["disk_free_bytes"], (int, float))
        and math.isfinite(float(sample["disk_free_bytes"]))
    ]
    return {
        "total_cpu_p95_pct": float(np.percentile(cpu, 95)) if cpu else None,
        "min_free_memory_gib": min(memory) / 1024 ** 3 if memory else None,
        "min_free_disk_gib": min(disk) / 1024 ** 3 if disk else None,
        "sample_count": len(samples),
        "has_cpu_memory_disk": complete,
    }
