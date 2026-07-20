import math
from types import SimpleNamespace

from slam_dynamic_filter.bag_reader import EvidenceRecord, Pose2D
from slam_dynamic_filter.metrics import (
    compute_compensated_residuals,
    compute_odom_tf_metrics,
    compute_profile_metrics,
    compute_scan_metrics,
    count_timestamp_reversals,
)


def stamp(stamp_ns):
    return SimpleNamespace(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )


def scan(stamp_ns, ranges):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(stamp_ns)),
        angle_min=-0.2,
        angle_increment=0.1,
        range_min=0.5,
        range_max=20.0,
        ranges=list(ranges),
    )


def test_stationary_scan_mad_and_rate_with_pose_delta_fallback():
    scans = [
        scan(i * 100_000_000, [2.0, 2.01, float("inf"), 3.0, 3.01])
        for i in range(12)
    ]
    odom = [Pose2D(i * 100_000_000, 0.0, 0.0, 0.0) for i in range(12)]
    metrics = compute_scan_metrics(scans, odom)
    assert metrics.beam_count == 5
    assert 9.5 <= metrics.rate_hz <= 10.5
    assert metrics.stationary_stable_bins == 4
    assert metrics.stationary_range_mad_p95_m <= 0.01


def test_odom_twist_excludes_moving_scans_from_stationary_bins():
    scans = [
        scan(i * 100_000_000, [2.0, float("inf") if i < 2 else 3.0])
        for i in range(10)
    ]
    odom = [
        Pose2D(
            i * 100_000_000,
            0.0,
            0.0,
            0.0,
            vx=0.0 if i < 2 else 0.2,
            vy=0.0,
            wz=0.0,
        )
        for i in range(10)
    ]
    metrics = compute_scan_metrics(scans, odom)
    assert metrics.stationary_stable_bins == 1


def test_pose_delta_excludes_motion_when_twist_is_unavailable():
    scans = [
        scan(i * 100_000_000, [2.0, float("inf") if i < 2 else 3.0])
        for i in range(10)
    ]
    odom = [
        Pose2D(i * 100_000_000, 0.0 if i < 2 else 0.02 * (i - 1), 0.0, 0.0)
        for i in range(10)
    ]
    metrics = compute_scan_metrics(scans, odom)
    assert metrics.stationary_stable_bins == 1


def test_pose_delta_excludes_motion_when_twist_is_finite_but_stale():
    scans = [
        scan(i * 100_000_000, [2.0, float("inf") if i < 2 else 3.0])
        for i in range(10)
    ]
    odom = [
        Pose2D(
            i * 100_000_000,
            0.0 if i < 2 else 0.02 * (i - 1),
            0.0,
            0.0,
            vx=0.0,
            vy=0.0,
            wz=0.0,
        )
        for i in range(10)
    ]
    metrics = compute_scan_metrics(scans, odom)
    assert metrics.stationary_stable_bins == 1


def test_yaw_wrap_tf_consistency_and_jumps():
    odom = [
        Pose2D(0, 0.0, 0.0, math.radians(179)),
        Pose2D(100, 0.1, 0.0, math.radians(-179)),
    ]
    tf = [
        Pose2D(0, 0.01, 0.0, math.radians(178)),
        Pose2D(100, 0.11, 0.0, math.radians(-178)),
    ]
    metrics = compute_odom_tf_metrics(odom, tf)
    assert metrics.tf_coverage == 1.0
    assert metrics.position_p95_m < 0.02
    assert metrics.yaw_p95_rad < 0.03
    assert metrics.yaw_jump_max_rad < 0.05


def test_compensated_residual_static_wall_overlap():
    scans = [
        scan(0, [2.0, 2.0, 2.0, 2.0]),
        scan(100_000_000, [2.0, 2.01, 2.0, 2.0]),
    ]
    odom = [
        Pose2D(0, 0.0, 0.0, 0.0),
        Pose2D(100_000_000, 0.0, 0.0, 0.0),
    ]
    metrics = compute_compensated_residuals(scans, odom)
    assert metrics.compared_pairs == 1
    assert metrics.nn_residual_p95_m < 0.03
    assert metrics.overlap_median == 1.0


def test_timestamp_reversal_uses_original_bag_sequence_and_profile_metrics():
    records = [
        EvidenceRecord("/scan_raw", 1, 1, 1, None, False),
        EvidenceRecord("/scan_raw", 2, 2, 0, None, False),
    ]
    assert count_timestamp_reversals(records)["/scan_raw"] == 1

    def transform(parent, child, stamp_ns):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=parent, stamp=stamp(stamp_ns)),
            child_frame_id=child,
        )

    static_records = [
        EvidenceRecord(
            "/tf_static",
            20,
            1,
            0,
            SimpleNamespace(transforms=[transform("base_link", "livox_frame", 20)]),
            False,
        ),
        EvidenceRecord(
            "/tf_static",
            10,
            2,
            1,
            SimpleNamespace(transforms=[transform("map", "odom", 10)]),
            False,
        ),
    ]
    assert count_timestamp_reversals(static_records)["/tf_static"] == 0
    static_records.append(
        EvidenceRecord(
            "/tf_static",
            15,
            3,
            2,
            SimpleNamespace(transforms=[transform("base_link", "livox_frame", 15)]),
            False,
        )
    )
    assert count_timestamp_reversals(static_records)["/tf_static"] == 1
    profile = compute_profile_metrics([
        {
            "total_cpu_percent": 50.0,
            "available_memory_bytes": 2 * 1024 ** 3,
            "disk_free_bytes": 30 * 1024 ** 3,
        },
        {
            "total_cpu_percent": 80.0,
            "available_memory_bytes": 1 * 1024 ** 3,
            "disk_free_bytes": 20 * 1024 ** 3,
        },
    ])
    assert profile["total_cpu_p95_pct"] >= 78.0
    assert profile["has_cpu_memory_disk"] is True
    incomplete = compute_profile_metrics([{"total_cpu_percent": 20.0}])
    assert incomplete["has_cpu_memory_disk"] is False
    assert incomplete["min_free_memory_gib"] is None
