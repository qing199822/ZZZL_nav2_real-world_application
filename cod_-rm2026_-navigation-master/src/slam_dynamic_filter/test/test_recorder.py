import json
import pathlib
import signal
import subprocess
from datetime import datetime, timezone

import pytest

from slam_dynamic_filter.constants import EXIT_RECORD_FAILED, EXIT_SUCCESS
from slam_dynamic_filter.metadata import create_session_paths
from slam_dynamic_filter.recorder import (
    RecordOptions,
    build_rosbag_argv,
    collect_system_sample,
    record_session,
    scenario_phase,
)


def test_rosbag_argv_is_direct_and_locked(tmp_path):
    paths = create_session_paths(tmp_path, "20260720T010203Z-static-wall-motion")
    argv = build_rosbag_argv(paths)
    assert argv[:4] == ["ros2", "bag", "record", "-o"]
    assert str(paths.bag) in argv
    assert "--storage" in argv and "sqlite3" in argv
    assert "--compression-mode" in argv and "file" in argv
    assert "--compression-format" in argv and "zstd" in argv
    assert "--max-bag-size" in argv and str(2 * 1024 * 1024 * 1024) in argv
    assert "/scan" not in argv
    assert all(token not in argv for token in ["bash", "-c"])
    assert "/scan" in build_rosbag_argv(paths, include_scan_baseline=True)


def test_duration_bounds_and_phase_prompts():
    with pytest.raises(ValueError):
        RecordOptions(duration_sec=0)
    with pytest.raises(ValueError):
        RecordOptions(duration_sec=301)
    assert scenario_phase(5).startswith("0-20")
    assert scenario_phase(105).startswith("100-120")


def test_system_sample_contains_gate_required_fields(tmp_path):
    sample = collect_system_sample(tmp_path)
    assert {
        "utc",
        "monotonic_sec",
        "total_cpu_percent",
        "per_core_cpu_percent",
        "available_memory_bytes",
        "disk_free_bytes",
        "processes",
        "thermal",
    } <= sample.keys()


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class FakeBagProcess:
    def __init__(self, argv, create_metadata=True):
        self.argv = argv
        self.returncode = None
        self.signals = []
        self.create_metadata = create_metadata

    def poll(self):
        return self.returncode

    def send_signal(self, sent_signal):
        self.signals.append(sent_signal)
        if self.create_metadata:
            output = pathlib.Path(self.argv[self.argv.index("-o") + 1])
            output.mkdir()
            (output / "metadata.yaml").write_text(
                "rosbag2_bagfile_information: {}\n",
                encoding="utf-8",
            )
        self.returncode = 0

    def wait(self, timeout=None):
        return self.returncode


def _git_runner(argv):
    assert argv == ["git", "rev-parse", "HEAD"]
    return subprocess.CompletedProcess(argv, 0, stdout="abc123\n", stderr="")


def test_record_session_freezes_metadata_profiles_and_stops_with_sigint(tmp_path, capsys):
    thresholds = pathlib.Path(__file__).parents[1] / "config" / "stage0_thresholds.yaml"
    clock = FakeClock()
    processes = []

    def popen(argv, **kwargs):
        assert kwargs.get("shell") is not True
        process = FakeBagProcess(argv)
        processes.append(process)
        return process

    result = record_session(
        RecordOptions(duration_sec=1),
        _git_runner,
        popen,
        output_root=tmp_path,
        thresholds_path=thresholds,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        utc_now=lambda: datetime(2026, 7, 20, 1, 2, 3, tzinfo=timezone.utc),
        sample_collector=lambda _: {
            "utc": "2026-07-20T01:02:03+00:00",
            "monotonic_sec": clock.monotonic(),
            "total_cpu_percent": 10.0,
            "per_core_cpu_percent": [10.0],
            "available_memory_bytes": 2 * 1024 ** 3,
            "disk_free_bytes": 30 * 1024 ** 3,
            "processes": [],
            "thermal": {},
        },
    )
    assert result == EXIT_SUCCESS
    assert processes[0].signals == [signal.SIGINT]
    sessions = list(tmp_path.iterdir())
    assert len(sessions) == 1
    session = sessions[0]
    assert (session / "metadata.json").is_file()
    assert (session / "record_manifest.json").is_file()
    assert (session / "system_profile.jsonl").is_file()
    assert (session / "config" / "stage0_thresholds.yaml").is_file()
    manifest = json.loads((session / "record_manifest.json").read_text(encoding="utf-8"))
    assert manifest["record_complete"] is True
    assert "SESSION 20260720T010203Z-static-wall-motion" in capsys.readouterr().out


def test_record_session_fails_closed_without_bag_metadata(tmp_path):
    thresholds = pathlib.Path(__file__).parents[1] / "config" / "stage0_thresholds.yaml"
    clock = FakeClock()

    def popen(argv, **kwargs):
        return FakeBagProcess(argv, create_metadata=False)

    result = record_session(
        RecordOptions(duration_sec=1),
        _git_runner,
        popen,
        output_root=tmp_path,
        thresholds_path=thresholds,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        utc_now=lambda: datetime(2026, 7, 20, 1, 2, 3, tzinfo=timezone.utc),
        sample_collector=lambda _: {
            "total_cpu_percent": 1.0,
            "available_memory_bytes": 2 * 1024 ** 3,
            "disk_free_bytes": 30 * 1024 ** 3,
        },
    )
    assert result == EXIT_RECORD_FAILED
