from __future__ import annotations

import json
import os
import pathlib
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Iterable, Sequence

import psutil

from .constants import (
    EXIT_RECORD_FAILED,
    EXIT_SUCCESS,
    OPTIONAL_TOPIC_TYPES,
    REQUIRED_TOPIC_TYPES,
    SCENARIO_ID,
    evidence_output_root,
)
from .metadata import (
    SessionPaths,
    atomic_write_json,
    create_session_paths,
    frozen_metadata,
    sha256_file,
)


@dataclass(frozen=True)
class RecordOptions:
    duration_sec: int = 120
    scenario: str = SCENARIO_ID
    include_scan_baseline: bool = False

    def __post_init__(self) -> None:
        if self.duration_sec < 1 or self.duration_sec > 300:
            raise ValueError("duration_sec must be between 1 and 300")
        if self.scenario != SCENARIO_ID:
            raise ValueError(f"unsupported scenario: {self.scenario}")


def build_rosbag_argv(paths: SessionPaths, include_scan_baseline: bool = False) -> list[str]:
    topics = list(REQUIRED_TOPIC_TYPES)
    if include_scan_baseline:
        topics.extend(OPTIONAL_TOPIC_TYPES)
    return [
        "ros2",
        "bag",
        "record",
        "-o",
        str(paths.bag),
        "--storage",
        "sqlite3",
        "--compression-mode",
        "file",
        "--compression-format",
        "zstd",
        "--max-bag-size",
        str(2 * 1024 * 1024 * 1024),
        *topics,
    ]


def scenario_phase(monotonic_elapsed_sec: float) -> str:
    phases = (
        (20, "0-20 stationary textured wall, no dynamic objects in 8 m zone"),
        (40, "20-40 slow yaw left/right <=0.4 rad/s, same wall visible"),
        (60, "40-60 slow forward/back <=0.3 m/s, keep >=1 m from wall"),
        (80, "60-80 slow lateral left/right <=0.3 m/s"),
        (100, "80-100 combined low-speed translation/yaw"),
        (120, "100-120 stationary near starting pose, no dynamic objects"),
    )
    for limit, label in phases:
        if monotonic_elapsed_sec < limit:
            return label
    return phases[-1][1]


def _thermal_sample() -> dict[str, list[dict[str, Any]]]:
    if not hasattr(psutil, "sensors_temperatures"):
        return {}
    result: dict[str, list[dict[str, Any]]] = {}
    for group, readings in (psutil.sensors_temperatures(fahrenheit=False) or {}).items():
        result[group] = [reading._asdict() for reading in readings]
    return result


def collect_system_sample(
    output_root: pathlib.Path,
    process_names: Iterable[str] = (),
) -> dict[str, Any]:
    selected_names = set(process_names)
    processes: list[dict[str, Any]] = []
    try:
        iterator = psutil.process_iter(["name", "pid", "memory_info", "cpu_percent"])
        for process in iterator:
            try:
                info = process.info
                if selected_names and info.get("name") not in selected_names:
                    continue
                memory_info = info.get("memory_info")
                processes.append({
                    "pid": info.get("pid"),
                    "name": info.get("name"),
                    "rss_bytes": memory_info.rss if memory_info else None,
                    "cpu_percent": info.get("cpu_percent"),
                })
            except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
                continue
    except (psutil.Error, OSError):
        processes = []
    return {
        "utc": datetime.now(timezone.utc).isoformat(),
        "monotonic_sec": time.monotonic(),
        "total_cpu_percent": psutil.cpu_percent(interval=None),
        "per_core_cpu_percent": psutil.cpu_percent(interval=None, percpu=True),
        "available_memory_bytes": psutil.virtual_memory().available,
        "disk_free_bytes": psutil.disk_usage(output_root).free,
        "processes": processes,
        "thermal": _thermal_sample(),
    }


def default_thresholds_path() -> pathlib.Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        path = (
            pathlib.Path(get_package_share_directory("slam_dynamic_filter"))
            / "config"
            / "stage0_thresholds.yaml"
        )
        if path.is_file():
            return path
    except (ImportError, LookupError):
        pass
    source_path = pathlib.Path(__file__).resolve().parents[1] / "config" / "stage0_thresholds.yaml"
    if not source_path.is_file():
        raise FileNotFoundError("stage0_thresholds.yaml not found")
    return source_path


def _session_id(now: datetime, scenario: str) -> str:
    return f"{now.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{scenario}"


def _append_json_line(stream: Any, payload: dict[str, Any]) -> None:
    stream.write(json.dumps(payload, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
    stream.flush()
    os.fsync(stream.fileno())


def record_session(
    options: RecordOptions,
    run: Callable[[Sequence[str]], subprocess.CompletedProcess[str]],
    popen_factory: Callable[..., subprocess.Popen],
    output_root: pathlib.Path | None = None,
    *,
    thresholds_path: pathlib.Path | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    utc_now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
    sample_collector: Callable[[pathlib.Path], dict[str, Any]] = collect_system_sample,
) -> int:
    root = output_root if output_root is not None else evidence_output_root()
    threshold_source = thresholds_path if thresholds_path is not None else default_thresholds_path()
    process: Any = None
    paths: SessionPaths | None = None
    manifest: dict[str, Any] | None = None
    try:
        git_result = run(["git", "rev-parse", "HEAD"])
        if git_result.returncode != 0 or not git_result.stdout.strip():
            return EXIT_RECORD_FAILED
        git_sha = git_result.stdout.strip()
        config_hash = sha256_file(threshold_source)
        now = utc_now()
        paths = create_session_paths(root, _session_id(now, options.scenario))
        frozen_config = paths.config / "stage0_thresholds.yaml"
        shutil.copyfile(threshold_source, frozen_config)
        metadata = frozen_metadata(paths.root.name, options.scenario, git_sha, config_hash)
        atomic_write_json(paths.metadata_json, metadata)
        manifest = {
            **metadata,
            "duration_sec": options.duration_sec,
            "include_scan_baseline": options.include_scan_baseline,
            "required_topics": dict(REQUIRED_TOPIC_TYPES),
            "record_complete": False,
        }
        atomic_write_json(paths.record_manifest_json, manifest)
        print(f"SESSION {paths.root.name}", flush=True)

        argv = build_rosbag_argv(paths, options.include_scan_baseline)
        process = popen_factory(argv)
        start = monotonic()
        last_sample_second = -1
        last_phase = ""
        with paths.system_profile_jsonl.open("x", encoding="utf-8") as profile_stream:
            while True:
                elapsed = max(0.0, monotonic() - start)
                if process.poll() is not None:
                    raise RuntimeError("rosbag process exited before requested duration")
                phase = scenario_phase(elapsed)
                if phase != last_phase:
                    print(f"SCENARIO {phase}", flush=True)
                    last_phase = phase
                sample_second = int(elapsed)
                if sample_second > last_sample_second:
                    _append_json_line(profile_stream, sample_collector(root))
                    last_sample_second = sample_second
                if elapsed >= options.duration_sec:
                    break
                sleep(min(0.2, options.duration_sec - elapsed))

        process.send_signal(signal.SIGINT)
        return_code = process.wait(timeout=30)
        if return_code != 0:
            raise RuntimeError(f"rosbag process returned {return_code}")
        if not (paths.bag / "metadata.yaml").is_file():
            raise RuntimeError("bag metadata.yaml missing after recording")
        manifest.update({
            "record_complete": True,
            "finished_utc": utc_now().astimezone(timezone.utc).isoformat(),
        })
        atomic_write_json(paths.record_manifest_json, manifest)
        return EXIT_SUCCESS
    except (OSError, RuntimeError, ValueError):
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)
            except (OSError, subprocess.SubprocessError):
                pass
        if paths is not None and manifest is not None:
            manifest["record_complete"] = False
            try:
                atomic_write_json(paths.record_manifest_json, manifest)
            except OSError:
                pass
        return EXIT_RECORD_FAILED
