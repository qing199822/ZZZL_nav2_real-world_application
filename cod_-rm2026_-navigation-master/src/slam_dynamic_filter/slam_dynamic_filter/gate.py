from __future__ import annotations

import math
import pathlib
from dataclasses import asdict, dataclass
from typing import Any, Mapping

import yaml

from .constants import SCHEMA_VERSION, SESSION_ID_RE
from .metadata import SessionPaths, atomic_write_json

THRESHOLD_KEYS = (
    "min_duration_sec",
    "scan_rate_hz_min",
    "scan_rate_hz_max",
    "scan_rate_cv_max",
    "expected_beam_count",
    "min_stationary_stable_bins",
    "stationary_range_mad_p95_max_m",
    "tf_coverage_min",
    "odom_tf_position_p95_max_m",
    "odom_tf_yaw_p95_max_rad",
    "odom_translation_jump_max_m",
    "odom_yaw_jump_max_rad",
    "compensated_nn_residual_p95_max_m",
    "compensated_overlap_min",
    "timestamp_reversal_max",
    "total_cpu_p95_max_pct",
    "min_free_memory_gib",
    "min_free_disk_gib",
)


@dataclass(frozen=True)
class Check:
    name: str
    measured: Any
    operator: str
    threshold: Any
    passed: bool
    reason: str


def _validate_thresholds(thresholds: Mapping[str, Any]) -> None:
    if thresholds.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("threshold schema_version must be 1")
    missing = [key for key in THRESHOLD_KEYS if key not in thresholds]
    if missing:
        raise ValueError(f"threshold keys missing: {', '.join(missing)}")
    for key in THRESHOLD_KEYS:
        value = thresholds[key]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise ValueError(f"threshold {key} must be a finite number")
    if not isinstance(thresholds["expected_beam_count"], int):
        raise ValueError("expected_beam_count must be an integer")


def load_thresholds(path: pathlib.Path) -> dict[str, Any]:
    try:
        with pathlib.Path(path).open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid threshold YAML: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("threshold YAML root must be a mapping")
    _validate_thresholds(data)
    return data


def _value(metrics: Mapping[str, Any], dotted_path: str) -> Any:
    current: Any = metrics
    for part in dotted_path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return None
        current = current[part]
    return current


def _is_unmeasured(value: Any) -> bool:
    return value is None or (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and not math.isfinite(float(value))
    )


def _make_check(
    name: str,
    measured: Any,
    operator: str,
    threshold: Any,
) -> Check:
    if _is_unmeasured(measured):
        return Check(name, measured, operator, threshold, False, "missing or unmeasured")
    try:
        passed = {
            ">=": measured >= threshold,
            "<=": measured <= threshold,
            "==": measured == threshold,
        }[operator]
    except (KeyError, TypeError, ValueError):
        return Check(name, measured, operator, threshold, False, "invalid measured value")
    return Check(
        name,
        measured,
        operator,
        threshold,
        bool(passed),
        "ok" if passed else "threshold failed",
    )


def _metric_check(
    metrics: Mapping[str, Any],
    name: str,
    path: str,
    operator: str,
    threshold: Any,
) -> Check:
    return _make_check(name, _value(metrics, path), operator, threshold)


def evaluate_gate(
    session_id: str,
    config_hash: str,
    git_sha: str,
    metrics: Mapping[str, Any],
    thresholds: Mapping[str, Any],
) -> dict[str, Any]:
    _validate_thresholds(thresholds)
    specs = (
        ("min_duration_sec", "scan_stability.duration_sec", ">=", thresholds["min_duration_sec"]),
        ("scan_rate_hz_min", "scan_stability.rate_hz", ">=", thresholds["scan_rate_hz_min"]),
        ("scan_rate_hz_max", "scan_stability.rate_hz", "<=", thresholds["scan_rate_hz_max"]),
        ("scan_rate_cv_max", "scan_stability.rate_cv", "<=", thresholds["scan_rate_cv_max"]),
        (
            "expected_beam_count",
            "scan_stability.beam_count",
            "==",
            thresholds["expected_beam_count"],
        ),
        (
            "min_stationary_stable_bins",
            "scan_stability.stationary_stable_bins",
            ">=",
            thresholds["min_stationary_stable_bins"],
        ),
        (
            "stationary_range_mad_p95_max_m",
            "scan_stability.stationary_range_mad_p95_m",
            "<=",
            thresholds["stationary_range_mad_p95_max_m"],
        ),
        (
            "tf_coverage_min",
            "same_source_odom_consistency.tf_coverage",
            ">=",
            thresholds["tf_coverage_min"],
        ),
        (
            "odom_tf_position_p95_max_m",
            "same_source_odom_consistency.position_p95_m",
            "<=",
            thresholds["odom_tf_position_p95_max_m"],
        ),
        (
            "odom_tf_yaw_p95_max_rad",
            "same_source_odom_consistency.yaw_p95_rad",
            "<=",
            thresholds["odom_tf_yaw_p95_max_rad"],
        ),
        (
            "odom_translation_jump_max_m",
            "same_source_odom_consistency.translation_jump_max_m",
            "<=",
            thresholds["odom_translation_jump_max_m"],
        ),
        (
            "odom_yaw_jump_max_rad",
            "same_source_odom_consistency.yaw_jump_max_rad",
            "<=",
            thresholds["odom_yaw_jump_max_rad"],
        ),
        (
            "compensated_nn_residual_p95_max_m",
            "compensated_scan.nn_residual_p95_m",
            "<=",
            thresholds["compensated_nn_residual_p95_max_m"],
        ),
        (
            "compensated_overlap_min",
            "compensated_scan.overlap_median",
            ">=",
            thresholds["compensated_overlap_min"],
        ),
        (
            "timestamp_reversal_max",
            "timestamps.max_reversals",
            "<=",
            thresholds["timestamp_reversal_max"],
        ),
        (
            "total_cpu_p95_max_pct",
            "r5_profile.total_cpu_p95_pct",
            "<=",
            thresholds["total_cpu_p95_max_pct"],
        ),
        (
            "min_free_memory_gib",
            "r5_profile.min_free_memory_gib",
            ">=",
            thresholds["min_free_memory_gib"],
        ),
        (
            "min_free_disk_gib",
            "r5_profile.min_free_disk_gib",
            ">=",
            thresholds["min_free_disk_gib"],
        ),
    )
    checks = [
        _make_check("session_id_valid", bool(SESSION_ID_RE.fullmatch(session_id)), "==", True),
        _make_check(
            "config_hash_valid",
            isinstance(config_hash, str)
            and len(config_hash) == 64
            and all(character in "0123456789abcdef" for character in config_hash),
            "==",
            True,
        ),
        _make_check("git_sha_present", isinstance(git_sha, str) and bool(git_sha), "==", True),
        _metric_check(
            metrics,
            "r5_profile_complete",
            "r5_profile.has_cpu_memory_disk",
            "==",
            True,
        ),
        _metric_check(
            metrics,
            "stale_source_timestamp_max",
            "timestamps.stale_source_count",
            "<=",
            0,
        ),
        *[_metric_check(metrics, *spec) for spec in specs],
    ]
    blocked = [check.name for check in checks if not check.passed]
    return {
        "schema_version": SCHEMA_VERSION,
        "session_id": session_id,
        "config_hash": config_hash,
        "git_sha": git_sha,
        "same_source_odom": True,
        "scan_stability": metrics.get("scan_stability"),
        "same_source_odom_consistency": metrics.get("same_source_odom_consistency"),
        "compensated_scan": metrics.get("compensated_scan"),
        "timestamps": metrics.get("timestamps"),
        "r5_profile": metrics.get("r5_profile"),
        "checks": [asdict(check) for check in checks],
        "allow_enforce": not blocked,
        "blocked_codes": blocked,
    }


def write_reports_atomic(
    session_paths: SessionPaths,
    metrics: Mapping[str, Any],
    gate: Mapping[str, Any],
) -> None:
    atomic_write_json(session_paths.reports / "stage0_metrics.json", metrics)
    atomic_write_json(session_paths.c1_gate_json, gate)
