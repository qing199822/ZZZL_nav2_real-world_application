import json

from slam_dynamic_filter.gate import evaluate_gate, write_reports_atomic
from slam_dynamic_filter.metadata import create_session_paths


def thresholds():
    return {
        "schema_version": 1,
        "min_duration_sec": 110.0,
        "scan_rate_hz_min": 8.0,
        "scan_rate_hz_max": 12.0,
        "scan_rate_cv_max": 0.15,
        "expected_beam_count": 723,
        "min_stationary_stable_bins": 100,
        "stationary_range_mad_p95_max_m": 0.08,
        "tf_coverage_min": 0.999,
        "odom_tf_position_p95_max_m": 0.05,
        "odom_tf_yaw_p95_max_rad": 0.05,
        "odom_translation_jump_max_m": 0.50,
        "odom_yaw_jump_max_rad": 0.50,
        "compensated_nn_residual_p95_max_m": 0.20,
        "compensated_overlap_min": 0.70,
        "timestamp_reversal_max": 0,
        "total_cpu_p95_max_pct": 75.0,
        "min_free_memory_gib": 1.0,
        "min_free_disk_gib": 20.0,
    }


def passing_metrics():
    return {
        "scan_stability": {
            "duration_sec": 120.0,
            "rate_hz": 10.0,
            "rate_cv": 0.02,
            "beam_count": 723,
            "stationary_stable_bins": 250,
            "stationary_range_mad_p95_m": 0.03,
        },
        "same_source_odom_consistency": {
            "tf_coverage": 1.0,
            "position_p95_m": 0.02,
            "yaw_p95_rad": 0.02,
            "translation_jump_max_m": 0.1,
            "yaw_jump_max_rad": 0.1,
        },
        "compensated_scan": {
            "nn_residual_p95_m": 0.08,
            "overlap_median": 0.9,
            "compared_pairs": 100,
        },
        "timestamps": {
            "max_reversals": 0,
            "stale_source_count": 0,
            "bag_time_fallback_count": 0,
        },
        "r5_profile": {
            "total_cpu_p95_pct": 55.0,
            "min_free_memory_gib": 2.0,
            "min_free_disk_gib": 30.0,
            "sample_count": 120,
            "has_cpu_memory_disk": True,
        },
    }


def test_gate_passes_complete_metrics():
    gate = evaluate_gate(
        "20260720T010203Z-static-wall-motion",
        "a" * 64,
        "abc123",
        passing_metrics(),
        thresholds(),
    )
    assert gate["schema_version"] == 1
    assert gate["same_source_odom"] is True
    assert gate["allow_enforce"] is True
    assert gate["blocked_codes"] == []
    assert all(set(check) == {
        "name", "measured", "operator", "threshold", "passed", "reason"
    } for check in gate["checks"])


def test_gate_blocks_threshold_failure():
    metrics = passing_metrics()
    metrics["scan_stability"]["rate_hz"] = 15.0
    gate = evaluate_gate(
        "20260720T010203Z-static-wall-motion",
        "a" * 64,
        "abc123",
        metrics,
        thresholds(),
    )
    assert gate["allow_enforce"] is False
    assert "scan_rate_hz_max" in gate["blocked_codes"]


def test_gate_fail_closed_on_missing_metric_profile_or_stale_timestamp():
    metrics = passing_metrics()
    del metrics["r5_profile"]["min_free_disk_gib"]
    metrics["r5_profile"]["has_cpu_memory_disk"] = False
    metrics["timestamps"]["stale_source_count"] = 1
    gate = evaluate_gate(
        "20260720T010203Z-static-wall-motion",
        "a" * 64,
        "abc123",
        metrics,
        thresholds(),
    )
    assert gate["allow_enforce"] is False
    expected_blocks = {
        "min_free_disk_gib",
        "r5_profile_complete",
        "stale_source_timestamp_max",
    }
    assert expected_blocks <= set(gate["blocked_codes"])


def test_reports_are_written_as_atomic_stable_json(tmp_path):
    paths = create_session_paths(tmp_path, "20260720T010203Z-static-wall-motion")
    metrics = passing_metrics()
    gate = evaluate_gate(paths.root.name, "a" * 64, "abc123", metrics, thresholds())
    write_reports_atomic(paths, metrics, gate)
    metrics_text = (paths.reports / "stage0_metrics.json").read_text(encoding="utf-8")
    assert metrics_text.endswith("\n")
    assert json.loads(metrics_text)["scan_stability"]["beam_count"] == 723
    assert json.loads(paths.c1_gate_json.read_text(encoding="utf-8"))["allow_enforce"] is True
