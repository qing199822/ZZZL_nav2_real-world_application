import json
import pathlib
import shutil
from types import SimpleNamespace

import pytest

from slam_dynamic_filter import cli
from slam_dynamic_filter.constants import (
    EXIT_ANALYSIS_INVALID,
    EXIT_C1_BLOCKED,
    EXIT_PREFLIGHT_BLOCKED,
    EXIT_RECORD_FAILED,
    EXIT_SUCCESS,
    EXIT_USAGE,
    REQUIRED_TOPIC_TYPES,
)
from slam_dynamic_filter.metadata import (
    atomic_write_json,
    create_session_paths,
    frozen_metadata,
    sha256_file,
)
from slam_dynamic_filter.metrics import (
    CompensatedResidualMetrics,
    OdomTfMetrics,
    ScanMetrics,
)


SESSION_ID = "20260720T010203Z-static-wall-motion"


def test_cli_has_doctor_record_analyze_and_usage_code_2():
    parser = cli.build_parser()
    assert parser.parse_args(["doctor"]).command == "doctor"
    assert parser.parse_args(["record", "--duration-sec", "120"]).duration_sec == 120
    assert parser.parse_args(["analyze", "--session", SESSION_ID]).command == "analyze"
    with pytest.raises(SystemExit) as exc:
        cli.main(["record", "--duration-sec", "301"])
    assert exc.value.code == EXIT_USAGE


def test_doctor_and_record_map_preflight_and_recorder_exit_codes(monkeypatch, tmp_path):
    blocked = SimpleNamespace(ok=False, errors=("missing topic",))
    monkeypatch.setattr(cli, "check_preflight", lambda *_: blocked)
    monkeypatch.setattr(cli, "evidence_output_root", lambda: tmp_path)
    assert cli.main(["doctor"]) == EXIT_PREFLIGHT_BLOCKED
    assert cli.main(["record"]) == EXIT_PREFLIGHT_BLOCKED

    passed = SimpleNamespace(ok=True, errors=())
    monkeypatch.setattr(cli, "check_preflight", lambda *_: passed)
    monkeypatch.setattr(cli, "record_session", lambda *_, **__: EXIT_RECORD_FAILED)
    assert cli.main(["record", "--duration-sec", "120"]) == EXIT_RECORD_FAILED


def _make_complete_session(tmp_path):
    paths = create_session_paths(tmp_path, SESSION_ID)
    threshold_source = pathlib.Path(__file__).parents[1] / "config" / "stage0_thresholds.yaml"
    frozen = paths.config / "stage0_thresholds.yaml"
    shutil.copyfile(threshold_source, frozen)
    config_hash = sha256_file(frozen)
    metadata = frozen_metadata(SESSION_ID, "static-wall-motion", "abc123", config_hash)
    atomic_write_json(paths.metadata_json, metadata)
    atomic_write_json(paths.record_manifest_json, {
        **metadata,
        "duration_sec": 120,
        "include_scan_baseline": False,
        "required_topics": dict(REQUIRED_TOPIC_TYPES),
        "record_complete": True,
    })
    paths.bag.mkdir()
    (paths.bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information: {}\n",
        encoding="utf-8",
    )
    paths.system_profile_jsonl.write_text(
        json.dumps({
            "total_cpu_percent": 50.0,
            "available_memory_bytes": 2 * 1024 ** 3,
            "disk_free_bytes": 30 * 1024 ** 3,
        }) + "\n",
        encoding="utf-8",
    )
    return paths


def _patch_passing_analysis(monkeypatch, blocked=False):
    monkeypatch.setattr(cli, "read_bag_records", lambda *_: [])
    monkeypatch.setattr(cli, "compute_scan_metrics", lambda *_: ScanMetrics(
        120.0, 15.0 if blocked else 10.0, 0.02, 723, 250, 0.03
    ))
    monkeypatch.setattr(cli, "compute_odom_tf_metrics", lambda *_: OdomTfMetrics(
        1.0, 0.02, 0.02, 0.1, 0.1
    ))
    monkeypatch.setattr(cli, "compute_compensated_residuals", lambda *_: CompensatedResidualMetrics(
        0.08, 0.9, 100
    ))


@pytest.mark.parametrize(
    ("blocked", "expected"),
    [(False, EXIT_SUCCESS), (True, EXIT_C1_BLOCKED)],
)
def test_analyze_validates_computes_writes_and_maps_gate_result(
    monkeypatch,
    tmp_path,
    blocked,
    expected,
):
    paths = _make_complete_session(tmp_path)
    monkeypatch.setattr(cli, "evidence_output_root", lambda: tmp_path)
    _patch_passing_analysis(monkeypatch, blocked=blocked)
    assert cli.main(["analyze", "--session", SESSION_ID]) == expected
    assert (paths.reports / "stage0_metrics.json").is_file()
    gate = json.loads(paths.c1_gate_json.read_text(encoding="utf-8"))
    assert gate["allow_enforce"] is (not blocked)


def test_analyze_rejects_missing_or_incomplete_session(monkeypatch, tmp_path):
    monkeypatch.setattr(cli, "evidence_output_root", lambda: tmp_path)
    assert cli.main(["analyze", "--session", SESSION_ID]) == EXIT_ANALYSIS_INVALID
    paths = _make_complete_session(tmp_path)
    manifest = json.loads(paths.record_manifest_json.read_text(encoding="utf-8"))
    manifest["record_complete"] = False
    atomic_write_json(paths.record_manifest_json, manifest)
    assert cli.main(["analyze", "--session", SESSION_ID]) == EXIT_ANALYSIS_INVALID


def test_analyze_rejects_artifact_symlink_escape(monkeypatch, tmp_path):
    paths = _make_complete_session(tmp_path)
    outside = tmp_path / "outside-session"
    outside.mkdir()
    paths.reports.rmdir()
    paths.reports.symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(cli, "evidence_output_root", lambda: tmp_path)
    _patch_passing_analysis(monkeypatch)
    assert cli.main(["analyze", "--session", SESSION_ID]) == EXIT_ANALYSIS_INVALID
    assert not (outside / "stage0_metrics.json").exists()
