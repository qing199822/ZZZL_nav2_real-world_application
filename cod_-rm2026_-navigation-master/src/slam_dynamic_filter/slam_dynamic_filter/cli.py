from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
from dataclasses import asdict
from typing import Any, Mapping, Sequence

from .bag_reader import (
    BagValidationError,
    EvidenceRecord,
    odom_to_pose2d,
    read_bag_records,
    tf_to_pose2d,
)
from .constants import (
    EXIT_ANALYSIS_INVALID,
    EXIT_C1_BLOCKED,
    EXIT_INTERNAL_ERROR,
    EXIT_PREFLIGHT_BLOCKED,
    EXIT_RECORD_FAILED,
    EXIT_SUCCESS,
    REQUIRED_TOPIC_TYPES,
    SESSION_ID_RE,
    evidence_output_root,
)
from .gate import evaluate_gate, load_thresholds, write_reports_atomic
from .metadata import (
    SessionPaths,
    canonical_child,
    load_json,
    sha256_file,
    validate_manifest,
)
from .metrics import (
    compute_compensated_residuals,
    compute_odom_tf_metrics,
    compute_profile_metrics,
    compute_scan_metrics,
    count_timestamp_reversals,
)
from .preflight import check_preflight
from .recorder import RecordOptions, record_session


class EvidenceValidationError(Exception):
    pass


def _duration_sec(value: str) -> int:
    try:
        duration = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("duration must be an integer") from exc
    if duration < 1 or duration > 300:
        raise argparse.ArgumentTypeError("duration must be between 1 and 300 seconds")
    return duration


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="sdfctl")
    subcommands = parser.add_subparsers(dest="command", required=True)
    subcommands.add_parser("doctor")
    record = subcommands.add_parser("record")
    record.add_argument("--duration-sec", type=_duration_sec, default=120)
    record.add_argument("--include-scan-baseline", action="store_true")
    analyze = subcommands.add_parser("analyze")
    analyze.add_argument("--session", required=True)
    analyze.add_argument("--thresholds", type=pathlib.Path, default=None)
    return parser


def _run_command(argv: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(argv),
        capture_output=True,
        text=True,
        check=False,
    )


def _existing_session_paths(output_root: pathlib.Path, session_id: str) -> SessionPaths:
    if not SESSION_ID_RE.fullmatch(session_id):
        raise EvidenceValidationError("invalid session id")
    root = canonical_child(output_root, pathlib.Path(output_root) / session_id)
    if not root.is_dir():
        raise EvidenceValidationError(f"session not found: {session_id}")

    def session_child(*parts: str) -> pathlib.Path:
        return canonical_child(root, root.joinpath(*parts))

    return SessionPaths(
        root=root,
        bag=session_child("bag"),
        config=session_child("config"),
        logs=session_child("logs"),
        reports=session_child("reports"),
        evidence=session_child("evidence"),
        metadata_json=session_child("metadata.json"),
        record_manifest_json=session_child("record_manifest.json"),
        system_profile_jsonl=session_child("system_profile.jsonl"),
        c1_gate_json=session_child("evidence", "c1_gate.json"),
    )


def _validate_session_artifacts(
    paths: SessionPaths,
    session_id: str,
    thresholds_override: pathlib.Path | None,
) -> tuple[dict[str, Any], dict[str, Any], pathlib.Path]:
    metadata = load_json(paths.metadata_json)
    manifest = load_json(paths.record_manifest_json)
    metadata_errors = validate_manifest(metadata)
    manifest_errors = validate_manifest(manifest)
    if metadata_errors or manifest_errors:
        errors = metadata_errors + manifest_errors
        raise EvidenceValidationError(f"invalid metadata or manifest: {'; '.join(errors)}")
    if metadata.get("session_id") != session_id or manifest.get("session_id") != session_id:
        raise EvidenceValidationError("session id does not match metadata")
    for key in ("scenario", "git_sha", "config_hash", "same_source_odom"):
        if metadata.get(key) != manifest.get(key):
            raise EvidenceValidationError(f"metadata/manifest mismatch: {key}")
    if manifest.get("record_complete") is not True:
        raise EvidenceValidationError("recording is incomplete")
    if manifest.get("required_topics") != dict(REQUIRED_TOPIC_TYPES):
        raise EvidenceValidationError("recorded required topic contract mismatch")
    duration = manifest.get("duration_sec")
    if isinstance(duration, bool) or not isinstance(duration, int) or not 1 <= duration <= 300:
        raise EvidenceValidationError("manifest duration invalid")
    if not (paths.bag / "metadata.yaml").is_file():
        raise EvidenceValidationError("bag metadata.yaml missing")
    if not paths.system_profile_jsonl.is_file():
        raise EvidenceValidationError("system_profile.jsonl missing")

    frozen_thresholds = paths.config / "stage0_thresholds.yaml"
    if not frozen_thresholds.is_file():
        raise EvidenceValidationError("frozen threshold config missing")
    expected_hash = metadata["config_hash"]
    if sha256_file(frozen_thresholds) != expected_hash:
        raise EvidenceValidationError("frozen threshold config hash mismatch")
    selected_thresholds = thresholds_override or frozen_thresholds
    if not selected_thresholds.is_file():
        raise EvidenceValidationError("selected threshold config missing")
    if sha256_file(selected_thresholds) != expected_hash:
        raise EvidenceValidationError("selected threshold config hash mismatch")
    return metadata, manifest, selected_thresholds


def _read_profile_samples(path: pathlib.Path) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                sample = json.loads(line)
            except json.JSONDecodeError as exc:
                raise EvidenceValidationError(
                    f"invalid system profile JSON at line {line_number}"
                ) from exc
            if not isinstance(sample, dict):
                raise EvidenceValidationError(
                    f"system profile line {line_number} is not an object"
                )
            samples.append(sample)
    return samples


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _compute_metrics(
    records: Sequence[EvidenceRecord],
    profile_samples: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    scans = [record.message for record in records if record.topic == "/scan_raw"]
    odom = [odom_to_pose2d(record.message) for record in records if record.topic == "/Odometry"]
    tf = [
        pose
        for record in records
        if record.topic == "/tf"
        for pose in tf_to_pose2d(record.message)
    ]
    reversals = count_timestamp_reversals(records)
    fallback_by_topic: dict[str, int] = {}
    for record in records:
        fallback_by_topic.setdefault(record.topic, 0)
        if record.used_bag_time_fallback:
            fallback_by_topic[record.topic] += 1
    stale_source_count = sum(
        1
        for record in records
        if record.topic != "/tf_static" and record.source_time_ns <= 0
    )
    metrics = {
        "scan_stability": asdict(compute_scan_metrics(scans, odom)),
        "same_source_odom_consistency": asdict(compute_odom_tf_metrics(odom, tf)),
        "compensated_scan": asdict(compute_compensated_residuals(scans, odom)),
        "timestamps": {
            "reversals_by_topic": reversals,
            "max_reversals": max(reversals.values(), default=0),
            "stale_source_count": stale_source_count,
            "bag_time_fallback_by_topic": fallback_by_topic,
            "bag_time_fallback_count": sum(fallback_by_topic.values()),
        },
        "r5_profile": compute_profile_metrics(profile_samples),
    }
    return _json_safe(metrics)


def _analyze(
    session_id: str,
    thresholds_override: pathlib.Path | None,
) -> dict[str, Any]:
    paths = _existing_session_paths(evidence_output_root(), session_id)
    metadata, _, thresholds_path = _validate_session_artifacts(
        paths,
        session_id,
        thresholds_override,
    )
    thresholds = load_thresholds(thresholds_path)
    records = read_bag_records(paths.bag)
    profile_samples = _read_profile_samples(paths.system_profile_jsonl)
    metrics = _compute_metrics(records, profile_samples)
    gate = evaluate_gate(
        session_id,
        metadata["config_hash"],
        metadata["git_sha"],
        metrics,
        thresholds,
    )
    write_reports_atomic(paths, metrics, gate)
    return gate


def _print_preflight(result: Any) -> None:
    if result.ok:
        print("PASS preflight: required topics and disk available")
    else:
        for error in result.errors:
            print(f"BLOCKED preflight: {error}")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "doctor":
            try:
                result = check_preflight(evidence_output_root(), _run_command)
            except (OSError, RuntimeError) as exc:
                print(f"BLOCKED preflight: {exc}")
                return EXIT_PREFLIGHT_BLOCKED
            _print_preflight(result)
            return EXIT_SUCCESS if result.ok else EXIT_PREFLIGHT_BLOCKED

        if args.command == "record":
            try:
                result = check_preflight(evidence_output_root(), _run_command)
            except (OSError, RuntimeError) as exc:
                print(f"BLOCKED preflight: {exc}")
                return EXIT_PREFLIGHT_BLOCKED
            _print_preflight(result)
            if not result.ok:
                return EXIT_PREFLIGHT_BLOCKED
            code = record_session(
                RecordOptions(
                    duration_sec=args.duration_sec,
                    include_scan_baseline=args.include_scan_baseline,
                ),
                _run_command,
                subprocess.Popen,
                output_root=evidence_output_root(),
            )
            if code == EXIT_SUCCESS:
                print("PASS record: evidence session complete")
                return code
            if code == EXIT_RECORD_FAILED:
                print("FAILED record: evidence session incomplete")
                return code
            return EXIT_INTERNAL_ERROR

        if args.command == "analyze":
            try:
                gate = _analyze(args.session, args.thresholds)
            except (
                BagValidationError,
                EvidenceValidationError,
                json.JSONDecodeError,
                OSError,
                ValueError,
            ) as exc:
                print(f"INVALID analysis: {exc}")
                return EXIT_ANALYSIS_INVALID
            if gate["allow_enforce"]:
                print("PASS C1: evidence gate passed")
                return EXIT_SUCCESS
            print("BLOCKED C1: valid evidence but thresholds failed")
            return EXIT_C1_BLOCKED
    except Exception as exc:
        print(f"INTERNAL ERROR: {exc}")
        return EXIT_INTERNAL_ERROR
    return EXIT_INTERNAL_ERROR
