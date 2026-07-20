# SLAM Dynamic Filter Stage 0 Evidence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 构建不改变现有 `/scan -> slam_toolbox` 路径的真机证据采集与确定性分析工具，验证 MID-360 二维投影和同源 Point-LIO 里程计是否足以支持后续动态目标跟踪。

**Architecture:** Add a future-compatible ROS 2 Humble ament_cmake package named `slam_dynamic_filter`, but Stage 0 contains evidence tooling only. A separate evidence launch starts the existing `pointcloud_to_laserscan_node` on `/livox/lidar_filtered` and publishes `/scan_raw`; it must never remap `/scan` or change `multiplenav_launch.py`. Python modules record a complete bag and frozen metadata, read the bag deterministically with rosbag2_py, compute scan/odom/TF/system metrics, and emit a versioned C1 gate JSON. No tracker, masker, safe map saver, offline SLAM rerun, or enforce mode in this plan.

**Tech Stack:** ROS 2 Humble, ament_cmake + ament_cmake_python, Python 3.10, rclpy, rosbag2_py, rosidl_runtime_py, NumPy, SciPy cKDTree, psutil, PyYAML, pytest/ament_pytest.

## Global Constraints

- Target machine AMD Ryzen 5 4500U, 6 cores / 6 threads, no discrete GPU.
- Current hardware uses Livox MID-360 built-in IMU only; no electrical IMU, wheel odometry, or Realsense requirement.
- `/Odometry` is same-source Point-LIO evidence, not independent ground truth.
- Stage 0 must not modify `cod_bringup/launch/multiplenav_launch.py`, `mapper_params_online_async.yaml`, `/scan`, `slam_toolbox`, Nav2 costmaps, or map autosave behavior.
- `/scan_raw` must consume `/livox/lidar_filtered`, use `target_frame=base_link`, z range `[0.1, 1.0]` m, angle range `[-pi, pi]`, `angle_increment=0.0087`, `scan_time=0.1`, range `[0.5, 20.0]`, and `use_inf=true`.
- Required bag topics and exact types: `/livox/lidar` and `/livox/lidar_filtered` = `sensor_msgs/msg/PointCloud2`; `/livox/imu` = `sensor_msgs/msg/Imu`; `/scan_raw` = `sensor_msgs/msg/LaserScan`; `/Odometry` = `nav_msgs/msg/Odometry`; `/tf` and `/tf_static` = `tf2_msgs/msg/TFMessage`. `/scan` is optional baseline only.
- Recorder duration default 120 s, maximum 300 s, minimum free disk 20 GiB, sqlite3 storage, zstd file compression, 2 GiB split size, fixed output root `$HOME/cod_mapping_sessions`.
- Use direct subprocess argument arrays; no `shell=True`, `bash -c`, or arbitrary output path.
- Schema version is integer `1`; all JSON is UTF-8, sorted keys, 2-space indent, newline terminated.
- Gate defaults to fail closed. Missing topic/type/config/hash/TF/metadata, stale or reversed timestamps, corrupt bag, or unmeasured required metric must set `allow_enforce=false`.
- Gate is evidence only; it does not add or enable an `enforce` runtime mode.
- Existing 7.84 s CustomMsg bag is explicitly invalid evidence.
- TDD: each production behavior must have a failing test first; no source implementation before its corresponding red test.
- Do not commit or push unless the user asks; although writing-plans usually shows commit steps, replace commit steps with review checkpoints (`git diff --check` and `git status --short`) because the user has not authorized commits.

---

## File Structure

Create package under `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/`:

- `package.xml`: ament/Python/runtime/test dependencies.
- `CMakeLists.txt`: install Python package, executable wrapper, launch/config; register pytest.
- `setup.cfg`: pytest/import path configuration only if required by ament_cmake_python; explain exact need.
- `slam_dynamic_filter/__init__.py`
- `slam_dynamic_filter/constants.py`: schema, required topics/types, immutable defaults, exit codes.
- `slam_dynamic_filter/metadata.py`: canonical root containment, SHA-256, frozen metadata and manifest validation.
- `slam_dynamic_filter/preflight.py`: ROS topic/type and disk preflight using injected command runner/disk provider.
- `slam_dynamic_filter/recorder.py`: session creation, non-shell ros2 bag process, timed scenario prompts, SIGINT shutdown, 1 Hz system profile sidecar, final completeness validation.
- `slam_dynamic_filter/bag_reader.py`: rosbag2_py deterministic deserialization and normalized evidence records.
- `slam_dynamic_filter/metrics.py`: scan geometry/rate/stationary-bin stability; odom/tf consistency; compensated consecutive-scan cKDTree residual; CPU/RSS/disk profile; no tracking.
- `slam_dynamic_filter/gate.py`: versioned threshold config, named checks, fail-closed decision and report rendering.
- `slam_dynamic_filter/cli.py`: `doctor`, `record`, `analyze` subcommands and stable exit codes.
- `scripts/sdfctl`: thin executable import wrapper.
- `launch/evidence_scan.launch.py`: only evidence projector `/livox/lidar_filtered -> /scan_raw`.
- `config/stage0_thresholds.yaml`: exact provisional thresholds.
- Tests: `test/test_constants.py`, `test/test_metadata.py`, `test/test_preflight.py`, `test/test_recorder.py`, `test/test_bag_reader.py`, `test/test_metrics.py`, `test/test_gate.py`, `test/test_cli.py`, `test/test_evidence_scan_launch.py`; fixture data `test/data/static_wall_records.json`, `test/data/invalid_manifest.json`.
- `README.md`: package-local command summary and Stage 0 limitation.

Outside package create:

- `docs/MAPPING_EVIDENCE_RUNBOOK.md`: operator sequence and 120-second scenario.
- Modify only `docs/QUICK_START.md` to link the runbook; do not modify source launch/config.

Exact artifact layout:

```text
~/cod_mapping_sessions/<session_id>/
  bag/
  config/
  logs/
  reports/
  evidence/
  metadata.json
  record_manifest.json
  system_profile.jsonl
  evidence/c1_gate.json
```

Session IDs regex: `^[0-9]{8}T[0-9]{6}Z-[a-z0-9][a-z0-9-]{0,47}$`. Scenario fixed initially to `static-wall-motion`.

Exact stable exit codes: `0` success/pass, `2` usage, `3` preflight blocked, `4` record failed/incomplete, `5` analysis invalid, `6` C1 blocked (valid evidence but thresholds fail), `7` internal error.

Exact provisional thresholds in `stage0_thresholds.yaml`; they are versioned evidence thresholds and must not be silently tuned:

```yaml
schema_version: 1
min_duration_sec: 110.0
scan_rate_hz_min: 8.0
scan_rate_hz_max: 12.0
scan_rate_cv_max: 0.15
expected_beam_count: 723
min_stationary_stable_bins: 100
stationary_range_mad_p95_max_m: 0.08
tf_coverage_min: 0.999
odom_tf_position_p95_max_m: 0.05
odom_tf_yaw_p95_max_rad: 0.05
odom_translation_jump_max_m: 0.50
odom_yaw_jump_max_rad: 0.50
compensated_nn_residual_p95_max_m: 0.20
compensated_overlap_min: 0.70
timestamp_reversal_max: 0
total_cpu_p95_max_pct: 75.0
min_free_memory_gib: 1.0
min_free_disk_gib: 20.0
```

Beam count is locked to actual converter behavior: `ranges_size = ceil((angle_max - angle_min) / angle_increment)`. With `angle_min=-3.1416`, `angle_max=3.1416`, `angle_increment=0.0087`, `ceil(6.2832 / 0.0087) = ceil(722.206...) = 723`. Stage 0 must add a red characterization test for this exact formula and must not suggest changing the converter.

Exact 120-second prompted scenario for recorder/runbook:

- `0-20 s`: stationary facing a textured static wall, no people/robots in 8 m measurement zone.
- `20-40 s`: slow yaw left then right, `<=0.4 rad/s`, keep same wall visible.
- `40-60 s`: slow forward then backward, `<=0.3 m/s`, keep `>=1 m` from wall.
- `60-80 s`: slow lateral left then right, `<=0.3 m/s`.
- `80-100 s`: combined low-speed translation/yaw, limits above.
- `100-120 s`: stationary at approximately starting pose; no dynamic objects.

Recorder prints phase transitions. Analyzer determines stationary/moving samples from odometry thresholds, not assumed timings. This C1 run is deliberately dynamic-object-free; a later move-stop bag is required for tracker calibration but is not part of Stage 0 gate.

Algorithms to implement and test:

- Bag record ordering is preserved by reader as `(source timestamp ns, topic priority, bag sequence)`, with topic priority constants; duplicate timestamps preserved.
- Normalize odom and TF to timestamped SE(2) pose records; interpolate `x/y` linearly and yaw on shortest angular path.
- For `/tf`, select `odom -> base_link`; `/tf_static` is recorded/validated but not used as dynamic pose.
- Stationary samples determined from odom twist if finite (`hypot(vx,vy)<=0.03 m/s`, `abs(wz)<=0.03 rad/s`), with pose-delta fallback over adjacent samples.
- Stationary scan stability: for bins finite in at least 80% of stationary scans, compute median absolute deviation of range; require at least 100 bins; metric is p95 across qualified bins.
- Odom/TF consistency: nearest/interpolated pose pairing within 50 ms; calculate position and wrapped yaw error p95 plus coverage.
- Compensated scan endpoints: finite scan ranges to base-frame `xy`, transform using interpolated odom pose to odom frame; compare consecutive scans no more than 200 ms apart using SciPy `cKDTree` nearest neighbors; overlap is fraction residual `<=0.20 m`; aggregate p95 residual over all matched nearest-neighbor residuals and median overlap across compared scan pairs.
- Timestamp reversals count per topic from source/header timestamps where present; bag timestamp fallback explicitly reported.
- System profile sidecar at 1 Hz includes UTC, monotonic seconds, total CPU percent, per-core CPU, available memory bytes, disk free bytes, process RSS/CPU for known node names when discoverable, thermal readings when available. Missing thermal is informational, not gate-blocking; missing CPU/memory/disk is blocking.
- C1 gate emits every check with `name`, `measured`, `operator`, `threshold`, `passed`, `reason`; top-level fields: `schema_version`, `session_id`, `config_hash`, `git_sha`, `same_source_odom=true`, `scan_stability`, `same_source_odom_consistency`, `r5_profile`, `checks`, `allow_enforce`, `blocked_codes`.
- `analyze` writes `reports/stage0_metrics.json` and `evidence/c1_gate.json` atomically via temp file + fsync + rename; source bag/config never modified.

## Task 1: Package Foundation, Constants, Threshold Config

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/package.xml`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/CMakeLists.txt`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/setup.cfg`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/__init__.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/constants.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/config/stage0_thresholds.yaml`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_constants.py`

**Interfaces**

```python
# slam_dynamic_filter/constants.py
SCHEMA_VERSION: int
SESSION_ID_RE: re.Pattern[str]
OUTPUT_ROOT_ENVLESS: str
SCENARIO_ID: str
REQUIRED_TOPIC_TYPES: Mapping[str, str]
OPTIONAL_TOPIC_TYPES: Mapping[str, str]
TOPIC_PRIORITY: Mapping[str, int]
EXIT_SUCCESS: int
EXIT_USAGE: int
EXIT_PREFLIGHT_BLOCKED: int
EXIT_RECORD_FAILED: int
EXIT_ANALYSIS_INVALID: int
EXIT_C1_BLOCKED: int
EXIT_INTERNAL_ERROR: int

def evidence_output_root(home: str | os.PathLike[str] | None = None) -> pathlib.Path: ...
def expected_scan_beams(angle_min: float = -3.1416, angle_max: float = 3.1416, angle_increment: float = 0.0087) -> int: ...
```

**Steps**

- [ ] Red: create `test/test_constants.py` first.

```python
from slam_dynamic_filter.constants import (
    EXIT_ANALYSIS_INVALID,
    EXIT_C1_BLOCKED,
    EXIT_INTERNAL_ERROR,
    EXIT_PREFLIGHT_BLOCKED,
    EXIT_RECORD_FAILED,
    EXIT_SUCCESS,
    EXIT_USAGE,
    REQUIRED_TOPIC_TYPES,
    SCHEMA_VERSION,
    SESSION_ID_RE,
    TOPIC_PRIORITY,
    expected_scan_beams,
)


def test_schema_topics_and_exit_codes_are_locked():
    assert SCHEMA_VERSION == 1
    assert REQUIRED_TOPIC_TYPES == {
        "/livox/lidar": "sensor_msgs/msg/PointCloud2",
        "/livox/lidar_filtered": "sensor_msgs/msg/PointCloud2",
        "/livox/imu": "sensor_msgs/msg/Imu",
        "/scan_raw": "sensor_msgs/msg/LaserScan",
        "/Odometry": "nav_msgs/msg/Odometry",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
    }
    assert (EXIT_SUCCESS, EXIT_USAGE, EXIT_PREFLIGHT_BLOCKED, EXIT_RECORD_FAILED,
            EXIT_ANALYSIS_INVALID, EXIT_C1_BLOCKED, EXIT_INTERNAL_ERROR) == (0, 2, 3, 4, 5, 6, 7)


def test_session_id_regex_and_topic_priority_are_stable():
    assert SESSION_ID_RE.fullmatch("20260720T010203Z-static-wall-motion")
    assert not SESSION_ID_RE.fullmatch("../escape")
    assert TOPIC_PRIORITY["/tf_static"] < TOPIC_PRIORITY["/tf"] < TOPIC_PRIORITY["/scan_raw"]


def test_converter_beam_count_characterization_is_723():
    assert expected_scan_beams() == 723
```

- [ ] Run red command from `/home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master`:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_constants.py
```

Expected red result: pytest reports import failure for `slam_dynamic_filter.constants`.

- [ ] Green: add package skeleton and constants.

```python
# slam_dynamic_filter/constants.py
from __future__ import annotations

import math
import os
import pathlib
import re
from types import MappingProxyType

SCHEMA_VERSION = 1
SESSION_ID_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z-[a-z0-9][a-z0-9-]{0,47}$")
OUTPUT_ROOT_ENVLESS = "cod_mapping_sessions"
SCENARIO_ID = "static-wall-motion"

REQUIRED_TOPIC_TYPES = MappingProxyType({
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/livox/lidar_filtered": "sensor_msgs/msg/PointCloud2",
    "/livox/imu": "sensor_msgs/msg/Imu",
    "/scan_raw": "sensor_msgs/msg/LaserScan",
    "/Odometry": "nav_msgs/msg/Odometry",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
})
OPTIONAL_TOPIC_TYPES = MappingProxyType({"/scan": "sensor_msgs/msg/LaserScan"})
TOPIC_PRIORITY = MappingProxyType({
    "/tf_static": 0,
    "/tf": 10,
    "/Odometry": 20,
    "/livox/imu": 30,
    "/livox/lidar": 40,
    "/livox/lidar_filtered": 50,
    "/scan_raw": 60,
    "/scan": 70,
})

EXIT_SUCCESS = 0
EXIT_USAGE = 2
EXIT_PREFLIGHT_BLOCKED = 3
EXIT_RECORD_FAILED = 4
EXIT_ANALYSIS_INVALID = 5
EXIT_C1_BLOCKED = 6
EXIT_INTERNAL_ERROR = 7


def evidence_output_root(home: str | os.PathLike[str] | None = None) -> pathlib.Path:
    base = pathlib.Path(home) if home is not None else pathlib.Path.home()
    return base / OUTPUT_ROOT_ENVLESS


def expected_scan_beams(
    angle_min: float = -3.1416,
    angle_max: float = 3.1416,
    angle_increment: float = 0.0087,
) -> int:
    return int(math.ceil((angle_max - angle_min) / angle_increment))
```

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(slam_dynamic_filter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/sdfctl DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_add_pytest_test(test_constants test/test_constants.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})

ament_package()
```

```xml
<!-- package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>slam_dynamic_filter</name>
  <version>0.0.1</version>
  <description>Stage 0 evidence tooling for MID-360 scan and same-source odometry analysis.</description>
  <maintainer email="robot@example.invalid">COD Robot Team</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>ament_cmake_python</build_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>rosbag2_py</exec_depend>
  <exec_depend>rosidl_runtime_py</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_msgs</exec_depend>
</package>
```

`setup.cfg` is only needed if pytest cannot import the source package through `APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}` under ament; if needed, make it minimal:

```ini
[tool:pytest]
testpaths = test
python_files = test_*.py
```

- [ ] Add `config/stage0_thresholds.yaml` exactly as shown in File Structure.
- [ ] Run green command:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_constants.py
colcon test-result --verbose
```

Expected pass names: `test_schema_topics_and_exit_codes_are_locked`, `test_session_id_regex_and_topic_priority_are_stable`, `test_converter_beam_count_characterization_is_723`.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Task 2: Metadata, Artifact Containment, Atomic JSON

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/metadata.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_metadata.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/data/invalid_manifest.json`

**Interfaces**

```python
# slam_dynamic_filter/metadata.py
@dataclass(frozen=True)
class SessionPaths:
    root: pathlib.Path
    bag: pathlib.Path
    config: pathlib.Path
    logs: pathlib.Path
    reports: pathlib.Path
    evidence: pathlib.Path
    metadata_json: pathlib.Path
    record_manifest_json: pathlib.Path
    system_profile_jsonl: pathlib.Path
    c1_gate_json: pathlib.Path

def canonical_child(root: pathlib.Path, child: pathlib.Path) -> pathlib.Path: ...
def create_session_paths(output_root: pathlib.Path, session_id: str) -> SessionPaths: ...
def sha256_file(path: pathlib.Path) -> str: ...
def atomic_write_json(path: pathlib.Path, payload: Mapping[str, Any]) -> None: ...
def load_json(path: pathlib.Path) -> dict[str, Any]: ...
def validate_manifest(manifest: Mapping[str, Any]) -> list[str]: ...
def frozen_metadata(session_id: str, scenario: str, git_sha: str, config_hash: str) -> dict[str, Any]: ...
```

**Steps**

- [ ] Red: create tests for traversal, symlink escape, stable JSON, SHA-256, schema, and session regex.

```python
import json
import os
import pathlib

import pytest

from slam_dynamic_filter.metadata import (
    atomic_write_json,
    canonical_child,
    create_session_paths,
    frozen_metadata,
    load_json,
    sha256_file,
    validate_manifest,
)


def test_canonical_child_rejects_traversal_and_symlink_escape(tmp_path):
    root = tmp_path / "root"
    root.mkdir()
    assert canonical_child(root, root / "ok").parent == root
    with pytest.raises(ValueError, match="outside output root"):
        canonical_child(root, root / ".." / "escape")
    outside = tmp_path / "outside"
    outside.mkdir()
    link = root / "link"
    link.symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="outside output root"):
        canonical_child(root, link / "x")


def test_session_paths_and_atomic_json_are_stable(tmp_path):
    paths = create_session_paths(tmp_path, "20260720T010203Z-static-wall-motion")
    assert paths.c1_gate_json == paths.evidence / "c1_gate.json"
    atomic_write_json(paths.metadata_json, {"b": 2, "a": 1})
    assert paths.metadata_json.read_text(encoding="utf-8") == '{\n  "a": 1,\n  "b": 2\n}\n'
    assert load_json(paths.metadata_json) == {"a": 1, "b": 2}


def test_sha256_and_manifest_validation(tmp_path):
    f = tmp_path / "data.txt"
    f.write_text("abc", encoding="utf-8")
    assert sha256_file(f) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    metadata = frozen_metadata("20260720T010203Z-static-wall-motion", "static-wall-motion", "abc123", "f" * 64)
    assert validate_manifest(metadata) == []
    invalid = json.loads(pathlib.Path("test/data/invalid_manifest.json").read_text(encoding="utf-8"))
    assert "schema_version" in " ".join(validate_manifest(invalid))
```

```json
{
  "schema_version": 2,
  "session_id": "../bad",
  "scenario": "wrong",
  "git_sha": "",
  "config_hash": "not-a-hash"
}
```

- [ ] Run red:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_metadata.py
```

Expected red result: import failure for `slam_dynamic_filter.metadata`.

- [ ] Green: implement metadata.

```python
from __future__ import annotations

import hashlib
import json
import os
import pathlib
import tempfile
from dataclasses import dataclass
from typing import Any, Mapping

from .constants import SCENARIO_ID, SCHEMA_VERSION, SESSION_ID_RE


@dataclass(frozen=True)
class SessionPaths:
    root: pathlib.Path
    bag: pathlib.Path
    config: pathlib.Path
    logs: pathlib.Path
    reports: pathlib.Path
    evidence: pathlib.Path
    metadata_json: pathlib.Path
    record_manifest_json: pathlib.Path
    system_profile_jsonl: pathlib.Path
    c1_gate_json: pathlib.Path


def canonical_child(root: pathlib.Path, child: pathlib.Path) -> pathlib.Path:
    root_real = root.resolve()
    child_real = child.resolve(strict=False)
    if os.path.commonpath([str(root_real), str(child_real)]) != str(root_real):
        raise ValueError(f"path outside output root: {child}")
    return child_real


def create_session_paths(output_root: pathlib.Path, session_id: str) -> SessionPaths:
    if not SESSION_ID_RE.fullmatch(session_id):
        raise ValueError(f"invalid session_id: {session_id}")
    root = canonical_child(output_root, output_root / session_id)
    paths = SessionPaths(
        root=root,
        bag=root / "bag",
        config=root / "config",
        logs=root / "logs",
        reports=root / "reports",
        evidence=root / "evidence",
        metadata_json=root / "metadata.json",
        record_manifest_json=root / "record_manifest.json",
        system_profile_jsonl=root / "system_profile.jsonl",
        c1_gate_json=root / "evidence" / "c1_gate.json",
    )
    for directory in (paths.root, paths.bag, paths.config, paths.logs, paths.reports, paths.evidence):
        directory.mkdir(parents=True, exist_ok=False)
    return paths


def sha256_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def atomic_write_json(path: pathlib.Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, path)
        dir_fd = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def load_json(path: pathlib.Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("JSON root must be object")
    return data


def frozen_metadata(session_id: str, scenario: str, git_sha: str, config_hash: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "session_id": session_id,
        "scenario": scenario,
        "git_sha": git_sha,
        "config_hash": config_hash,
        "same_source_odom": True,
    }


def validate_manifest(manifest: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if manifest.get("schema_version") != SCHEMA_VERSION:
        errors.append("schema_version must be 1")
    if not isinstance(manifest.get("session_id"), str) or not SESSION_ID_RE.fullmatch(manifest["session_id"]):
        errors.append("session_id invalid")
    if manifest.get("scenario") != SCENARIO_ID:
        errors.append("scenario invalid")
    if not isinstance(manifest.get("git_sha"), str) or not manifest["git_sha"]:
        errors.append("git_sha missing")
    config_hash = manifest.get("config_hash")
    if not isinstance(config_hash, str) or len(config_hash) != 64 or any(c not in "0123456789abcdef" for c in config_hash):
        errors.append("config_hash invalid")
    if manifest.get("same_source_odom") is not True:
        errors.append("same_source_odom must be true")
    return errors
```

- [ ] Register `test_metadata` in `CMakeLists.txt` using `ament_add_pytest_test(test_metadata test/test_metadata.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})`.
- [ ] Run green:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_metadata.py
colcon test-result --verbose
```

Expected pass names: `test_canonical_child_rejects_traversal_and_symlink_escape`, `test_session_paths_and_atomic_json_are_stable`, `test_sha256_and_manifest_validation`.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Task 3: Evidence Scan Launch

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/launch/evidence_scan.launch.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_evidence_scan_launch.py`

**Interfaces**

```python
# launch/evidence_scan.launch.py
def generate_launch_description() -> launch.LaunchDescription: ...
```

**Steps**

- [ ] Red: create launch structure test.

```python
import importlib.util
from pathlib import Path

from launch_ros.actions import Node


def _load_launch():
    path = Path("launch/evidence_scan.launch.py")
    spec = importlib.util.spec_from_file_location("evidence_scan_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_evidence_scan_projects_filtered_cloud_to_scan_raw_only():
    ld = _load_launch()
    nodes = [a for a in ld.entities if isinstance(a, Node)]
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == "pointcloud_to_laserscan"
    assert node.node_executable == "pointcloud_to_laserscan_node"
    assert ("/livox/lidar_filtered", "/cloud_in") in node.node_remappings
    assert ("/scan", "/scan_raw") in node.node_remappings
    assert all("/scan:=" not in str(item) for item in node.cmd)


def test_evidence_scan_parameters_are_locked():
    node = [a for a in _load_launch().entities if isinstance(a, Node)][0]
    params = node.node_parameters[0]
    assert params["target_frame"] == "base_link"
    assert params["min_height"] == 0.1
    assert params["max_height"] == 1.0
    assert params["angle_min"] == -3.1416
    assert params["angle_max"] == 3.1416
    assert params["angle_increment"] == 0.0087
    assert params["scan_time"] == 0.1
    assert params["range_min"] == 0.5
    assert params["range_max"] == 20.0
    assert params["use_inf"] is True
```

- [ ] Run red:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_evidence_scan_launch.py
```

Expected red result: missing `launch/evidence_scan.launch.py`.

- [ ] Green: add launch file exactly:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="sdf_evidence_pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("/livox/lidar_filtered", "/cloud_in"),
                ("/scan", "/scan_raw"),
            ],
            parameters=[{
                "target_frame": "base_link",
                "transform_tolerance": 0.01,
                "min_height": 0.1,
                "max_height": 1.0,
                "angle_min": -3.1416,
                "angle_max": 3.1416,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.5,
                "range_max": 20.0,
                "use_inf": True,
            }],
        )
    ])
```

- [ ] Register `test_evidence_scan_launch` in `CMakeLists.txt`.
- [ ] Run green:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_evidence_scan_launch.py
colcon test-result --verbose
```

Expected pass names: `test_evidence_scan_projects_filtered_cloud_to_scan_raw_only`, `test_evidence_scan_parameters_are_locked`.

- [ ] Manual launch check without touching production launch:

```bash
source install/setup.bash
ros2 launch slam_dynamic_filter evidence_scan.launch.py
```

Expected visible node: `sdf_evidence_pointcloud_to_laserscan`. Expected absent behavior: no edit to `cod_bringup/launch/multiplenav_launch.py`, no `/scan` remap in that file.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Task 4: Preflight and Recorder

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/preflight.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/recorder.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_preflight.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_recorder.py`

**Interfaces**

```python
# slam_dynamic_filter/preflight.py
@dataclass(frozen=True)
class PreflightResult:
    ok: bool
    errors: tuple[str, ...]
    topic_types: Mapping[str, str]
    free_disk_bytes: int

CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess[str]]
DiskProvider = Callable[[pathlib.Path], shutil._ntuple_diskusage]

def query_topic_types(run: CommandRunner) -> dict[str, str]: ...
def check_preflight(output_root: pathlib.Path, run: CommandRunner, disk_usage: DiskProvider = shutil.disk_usage) -> PreflightResult: ...

# slam_dynamic_filter/recorder.py
@dataclass(frozen=True)
class RecordOptions:
    duration_sec: int = 120
    scenario: str = "static-wall-motion"
    include_scan_baseline: bool = False

def build_rosbag_argv(paths: SessionPaths, include_scan_baseline: bool = False) -> list[str]: ...
def scenario_phase(monotonic_elapsed_sec: float) -> str: ...
def collect_system_sample(output_root: pathlib.Path, process_names: Iterable[str] = ()) -> dict[str, Any]: ...
def record_session(options: RecordOptions, run: CommandRunner, popen_factory: Callable[..., subprocess.Popen], output_root: pathlib.Path | None = None) -> int: ...
```

**Steps**

- [ ] Red: create preflight tests with injected fakes.

```python
import shutil
import subprocess

from slam_dynamic_filter.constants import REQUIRED_TOPIC_TYPES
from slam_dynamic_filter.preflight import check_preflight, query_topic_types


def test_query_topic_types_parses_ros2_output():
    def run(argv):
        assert argv == ["ros2", "topic", "list", "-t"]
        return subprocess.CompletedProcess(argv, 0, stdout="/scan_raw [sensor_msgs/msg/LaserScan]\n/Odometry [nav_msgs/msg/Odometry]\n", stderr="")

    assert query_topic_types(run)["/scan_raw"] == "sensor_msgs/msg/LaserScan"


def test_preflight_rejects_custommsg_and_low_disk(tmp_path):
    def run(argv):
        lines = [f"{topic} [{typ}]" for topic, typ in REQUIRED_TOPIC_TYPES.items()]
        lines[0] = "/livox/lidar [livox_ros_driver2/msg/CustomMsg]"
        return subprocess.CompletedProcess(argv, 0, stdout="\n".join(lines), stderr="")

    usage = shutil._ntuple_diskusage(total=30, used=20, free=10)
    result = check_preflight(tmp_path, run, disk_usage=lambda _: usage)
    assert not result.ok
    assert any("type mismatch /livox/lidar" in e for e in result.errors)
    assert any("free disk" in e for e in result.errors)
```

- [ ] Red: create recorder argv, duration, signal, profile, manifest tests.

```python
import pathlib

import pytest

from slam_dynamic_filter.metadata import create_session_paths
from slam_dynamic_filter.recorder import RecordOptions, build_rosbag_argv, collect_system_sample, scenario_phase


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


def test_duration_bounds_and_phase_prompts():
    with pytest.raises(ValueError):
        RecordOptions(duration_sec=301)
    assert scenario_phase(5).startswith("0-20")
    assert scenario_phase(105).startswith("100-120")


def test_system_sample_contains_gate_required_fields(tmp_path):
    sample = collect_system_sample(tmp_path)
    assert "utc" in sample
    assert "monotonic_sec" in sample
    assert "total_cpu_percent" in sample
    assert "per_core_cpu_percent" in sample
    assert "available_memory_bytes" in sample
    assert "disk_free_bytes" in sample
```

- [ ] Run red:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_preflight.py test/test_recorder.py
```

Expected red result: import failures for `preflight` and `recorder`.

- [ ] Green: implement preflight parser and recorder primitives.

```python
# preflight.py
from __future__ import annotations

import pathlib
import re
import shutil
import subprocess
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

from .constants import REQUIRED_TOPIC_TYPES

CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess[str]]
DiskProvider = Callable[[pathlib.Path], shutil._ntuple_diskusage]
MIN_FREE_DISK_BYTES = 20 * 1024 ** 3
TOPIC_LINE_RE = re.compile(r"^(?P<topic>/\S+)\s+\[(?P<type>[^\]]+)\]$")


@dataclass(frozen=True)
class PreflightResult:
    ok: bool
    errors: tuple[str, ...]
    topic_types: Mapping[str, str]
    free_disk_bytes: int


def query_topic_types(run: CommandRunner) -> dict[str, str]:
    completed = run(["ros2", "topic", "list", "-t"])
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or "ros2 topic list failed")
    result: dict[str, str] = {}
    for line in completed.stdout.splitlines():
        match = TOPIC_LINE_RE.match(line.strip())
        if match:
            result[match.group("topic")] = match.group("type")
    return result


def check_preflight(output_root: pathlib.Path, run: CommandRunner, disk_usage: DiskProvider = shutil.disk_usage) -> PreflightResult:
    topic_types = query_topic_types(run)
    usage = disk_usage(output_root)
    errors: list[str] = []
    for topic, expected in REQUIRED_TOPIC_TYPES.items():
        actual = topic_types.get(topic)
        if actual is None:
            errors.append(f"missing topic {topic}")
        elif actual != expected:
            errors.append(f"type mismatch {topic}: expected {expected}, got {actual}")
    if usage.free < MIN_FREE_DISK_BYTES:
        errors.append(f"free disk below 20 GiB: {usage.free}")
    return PreflightResult(not errors, tuple(errors), topic_types, usage.free)
```

```python
# recorder.py
from __future__ import annotations

import pathlib
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Iterable, Sequence

import psutil

from .constants import OPTIONAL_TOPIC_TYPES, REQUIRED_TOPIC_TYPES, SCENARIO_ID, evidence_output_root
from .metadata import SessionPaths


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
        "ros2", "bag", "record",
        "-o", str(paths.bag),
        "--storage", "sqlite3",
        "--compression-mode", "file",
        "--compression-format", "zstd",
        "--max-bag-size", str(2 * 1024 * 1024 * 1024),
        *topics,
    ]


def scenario_phase(monotonic_elapsed_sec: float) -> str:
    phases = [
        (20, "0-20 stationary textured wall, no dynamic objects in 8 m zone"),
        (40, "20-40 slow yaw left/right <=0.4 rad/s, same wall visible"),
        (60, "40-60 slow forward/back <=0.3 m/s, keep >=1 m from wall"),
        (80, "60-80 slow lateral left/right <=0.3 m/s"),
        (100, "80-100 combined low-speed translation/yaw"),
        (120, "100-120 stationary near starting pose, no dynamic objects"),
    ]
    for limit, label in phases:
        if monotonic_elapsed_sec < limit:
            return label
    return phases[-1][1]


def collect_system_sample(output_root: pathlib.Path, process_names: Iterable[str] = ()) -> dict[str, Any]:
    names = set(process_names)
    processes = []
    for proc in psutil.process_iter(["name", "pid", "memory_info", "cpu_percent"]):
        info = proc.info
        if not names or info.get("name") in names:
            rss = info["memory_info"].rss if info.get("memory_info") else None
            processes.append({"pid": info.get("pid"), "name": info.get("name"), "rss_bytes": rss, "cpu_percent": info.get("cpu_percent")})
    thermal = {}
    if hasattr(psutil, "sensors_temperatures"):
        thermal = psutil.sensors_temperatures(fahrenheit=False) or {}
    return {
        "utc": datetime.now(timezone.utc).isoformat(),
        "monotonic_sec": time.monotonic(),
        "total_cpu_percent": psutil.cpu_percent(interval=None),
        "per_core_cpu_percent": psutil.cpu_percent(interval=None, percpu=True),
        "available_memory_bytes": psutil.virtual_memory().available,
        "disk_free_bytes": psutil.disk_usage(output_root).free,
        "processes": processes,
        "thermal": thermal,
    }


def record_session(options: RecordOptions, run: Callable[[Sequence[str]], subprocess.CompletedProcess[str]], popen_factory: Callable[..., subprocess.Popen], output_root: pathlib.Path | None = None) -> int:
    raise NotImplementedError("Task 7 wires CLI; Task 4 tests and implements full process lifecycle before use")
```

- [ ] Extend recorder in the same task before green is accepted: `record_session` must create `SessionPaths`, write metadata/config copy/manifest, start `popen_factory(build_rosbag_argv(...))`, print phase transitions, append one JSON system profile row per second, send SIGINT on timeout, wait, and validate required bag metadata exists. Tests use fake Popen with `send_signal(signal.SIGINT)` assertion.
- [ ] Register tests in `CMakeLists.txt` and run:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_preflight.py test/test_recorder.py
colcon test-result --verbose
```

Expected pass names: `test_query_topic_types_parses_ros2_output`, `test_preflight_rejects_custommsg_and_low_disk`, `test_rosbag_argv_is_direct_and_locked`, `test_duration_bounds_and_phase_prompts`, `test_system_sample_contains_gate_required_fields`.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Task 5: Deterministic Bag Reader and Normalization

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/bag_reader.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_bag_reader.py`

**Interfaces**

```python
# slam_dynamic_filter/bag_reader.py
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

class BagValidationError(Exception): ...

def source_timestamp_ns(topic: str, msg: Any, bag_time_ns: int) -> tuple[int, bool]: ...
def read_bag_records(bag_dir: pathlib.Path, required_topic_types: Mapping[str, str] = REQUIRED_TOPIC_TYPES) -> list[EvidenceRecord]: ...
def sort_records(records: Iterable[EvidenceRecord]) -> list[EvidenceRecord]: ...
def odom_to_pose2d(msg: Any) -> Pose2D: ...
def tf_to_pose2d(msg: Any, parent: str = "odom", child: str = "base_link") -> list[Pose2D]: ...
def interpolate_pose(records: Sequence[Pose2D], stamp_ns: int, max_gap_ns: int | None = None) -> Pose2D | None: ...
```

**Steps**

- [ ] Red: create deterministic ordering and normalization tests using fake messages.

```python
import math
from types import SimpleNamespace

from slam_dynamic_filter.bag_reader import EvidenceRecord, Pose2D, interpolate_pose, sort_records


def test_sort_records_uses_source_time_priority_sequence_and_preserves_duplicates():
    records = [
        EvidenceRecord("/scan_raw", 100, 500, 2, "scan-a", False),
        EvidenceRecord("/tf", 100, 400, 0, "tf", False),
        EvidenceRecord("/scan_raw", 100, 300, 1, "scan-b", False),
    ]
    ordered = sort_records(records)
    assert [r.message for r in ordered] == ["tf", "scan-b", "scan-a"]


def test_interpolate_pose_uses_shortest_yaw_path():
    poses = [
        Pose2D(0, 0.0, 0.0, math.radians(170)),
        Pose2D(10, 10.0, 0.0, math.radians(-170)),
    ]
    mid = interpolate_pose(poses, 5)
    assert mid.x == 5.0
    assert abs(abs(mid.yaw) - math.pi) < 1e-9


def test_missing_pose_gap_returns_none():
    poses = [Pose2D(0, 0.0, 0.0, 0.0), Pose2D(100, 1.0, 0.0, 0.0)]
    assert interpolate_pose(poses, 50, max_gap_ns=20) is None
```

- [ ] Run red:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_bag_reader.py
```

Expected red result: missing `bag_reader`.

- [ ] Green: implement sorting, source timestamp extraction, odom/TF quaternion yaw normalization, interpolation, and rosbag2 reader adapter.

```python
from __future__ import annotations

import math
import pathlib
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence

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


class BagValidationError(Exception):
    pass


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def source_timestamp_ns(topic: str, msg: Any, bag_time_ns: int) -> tuple[int, bool]:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        return _stamp_ns(stamp), False
    return int(bag_time_ns), True


def sort_records(records: Iterable[EvidenceRecord]) -> list[EvidenceRecord]:
    return sorted(records, key=lambda r: (r.source_time_ns, TOPIC_PRIORITY.get(r.topic, 999), r.sequence))


def _yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_to_pose2d(msg: Any) -> Pose2D:
    pose = msg.pose.pose
    return Pose2D(_stamp_ns(msg.header.stamp), float(pose.position.x), float(pose.position.y), _yaw_from_quaternion(pose.orientation))


def tf_to_pose2d(msg: Any, parent: str = "odom", child: str = "base_link") -> list[Pose2D]:
    poses = []
    for transform in getattr(msg, "transforms", []):
        if transform.header.frame_id == parent and transform.child_frame_id == child:
            t = transform.transform.translation
            poses.append(Pose2D(_stamp_ns(transform.header.stamp), float(t.x), float(t.y), _yaw_from_quaternion(transform.transform.rotation)))
    return poses


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def interpolate_pose(records: Sequence[Pose2D], stamp_ns: int, max_gap_ns: int | None = None) -> Pose2D | None:
    if not records:
        return None
    ordered = sorted(records, key=lambda p: p.stamp_ns)
    if stamp_ns < ordered[0].stamp_ns or stamp_ns > ordered[-1].stamp_ns:
        return None
    for left, right in zip(ordered, ordered[1:]):
        if left.stamp_ns <= stamp_ns <= right.stamp_ns:
            gap = right.stamp_ns - left.stamp_ns
            if max_gap_ns is not None and gap > max_gap_ns:
                return None
            if gap == 0:
                return left
            alpha = (stamp_ns - left.stamp_ns) / gap
            yaw = _wrap_pi(left.yaw + alpha * _wrap_pi(right.yaw - left.yaw))
            return Pose2D(stamp_ns, left.x + alpha * (right.x - left.x), left.y + alpha * (right.y - left.y), yaw)
    return ordered[-1] if stamp_ns == ordered[-1].stamp_ns else None
```

- [ ] Implement `read_bag_records` with `rosbag2_py.SequentialReader`, `rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")`, `rosbag2_py.ConverterOptions("", "")`, `rosidl_runtime_py.utilities.get_message`, and `rclpy.serialization.deserialize_message`. Validate required topics and exact types before deserializing; raise `BagValidationError` on corrupt bag, missing type, missing topic, or deserialization failure.
- [ ] Register and run:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_bag_reader.py
colcon test-result --verbose
```

Expected pass names: `test_sort_records_uses_source_time_priority_sequence_and_preserves_duplicates`, `test_interpolate_pose_uses_shortest_yaw_path`, `test_missing_pose_gap_returns_none`.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Task 6: Pure Metrics

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/metrics.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_metrics.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/data/static_wall_records.json`

**Interfaces**

```python
# slam_dynamic_filter/metrics.py
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

def compute_scan_metrics(scan_records: Sequence[Any], odom_poses: Sequence[Pose2D]) -> ScanMetrics: ...
def compute_odom_tf_metrics(odom: Sequence[Pose2D], tf: Sequence[Pose2D]) -> OdomTfMetrics: ...
def compute_compensated_residuals(scans: Sequence[Any], odom: Sequence[Pose2D]) -> CompensatedResidualMetrics: ...
def count_timestamp_reversals(records: Sequence[EvidenceRecord]) -> dict[str, int]: ...
def compute_profile_metrics(samples: Sequence[Mapping[str, Any]]) -> dict[str, float | int | bool]: ...
```

**Steps**

- [ ] Red: create synthetic static wall fixtures and tests.

```python
import math
from types import SimpleNamespace

import numpy as np

from slam_dynamic_filter.bag_reader import EvidenceRecord, Pose2D
from slam_dynamic_filter.metrics import (
    compute_compensated_residuals,
    compute_odom_tf_metrics,
    compute_profile_metrics,
    compute_scan_metrics,
    count_timestamp_reversals,
)


def scan(stamp_ns, ranges):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
        angle_min=-0.2,
        angle_increment=0.1,
        range_min=0.5,
        range_max=20.0,
        ranges=list(ranges),
    )


def test_stationary_scan_mad_and_rate():
    scans = [scan(i * 100_000_000, [2.0, 2.01, float("inf"), 3.0, 3.01]) for i in range(12)]
    odom = [Pose2D(i * 100_000_000, 0.0, 0.0, 0.0) for i in range(12)]
    metrics = compute_scan_metrics(scans, odom)
    assert metrics.beam_count == 5
    assert 9.5 <= metrics.rate_hz <= 10.5
    assert metrics.stationary_stable_bins >= 4
    assert metrics.stationary_range_mad_p95_m <= 0.01


def test_yaw_wrap_tf_consistency_and_jumps():
    odom = [Pose2D(0, 0.0, 0.0, math.radians(179)), Pose2D(100, 0.1, 0.0, math.radians(-179))]
    tf = [Pose2D(0, 0.01, 0.0, math.radians(178)), Pose2D(100, 0.11, 0.0, math.radians(-178))]
    metrics = compute_odom_tf_metrics(odom, tf)
    assert metrics.tf_coverage == 1.0
    assert metrics.position_p95_m < 0.02
    assert metrics.yaw_p95_rad < 0.03
    assert metrics.yaw_jump_max_rad < 0.05


def test_compensated_residual_static_wall_overlap():
    scans = [scan(0, [2.0, 2.0, 2.0, 2.0]), scan(100_000_000, [2.0, 2.01, 2.0, 2.0])]
    odom = [Pose2D(0, 0.0, 0.0, 0.0), Pose2D(100_000_000, 0.0, 0.0, 0.0)]
    metrics = compute_compensated_residuals(scans, odom)
    assert metrics.compared_pairs == 1
    assert metrics.nn_residual_p95_m < 0.03
    assert metrics.overlap_median == 1.0


def test_timestamp_reversal_and_profile_metrics():
    records = [
        EvidenceRecord("/scan_raw", 2, 2, 0, None, False),
        EvidenceRecord("/scan_raw", 1, 1, 1, None, False),
    ]
    assert count_timestamp_reversals(records)["/scan_raw"] == 1
    profile = compute_profile_metrics([
        {"total_cpu_percent": 50.0, "available_memory_bytes": 2 * 1024 ** 3, "disk_free_bytes": 30 * 1024 ** 3},
        {"total_cpu_percent": 80.0, "available_memory_bytes": 1 * 1024 ** 3, "disk_free_bytes": 20 * 1024 ** 3},
    ])
    assert profile["total_cpu_p95_pct"] >= 78.0
    assert profile["has_cpu_memory_disk"] is True
```

- [ ] Run red:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_metrics.py
```

Expected red result: missing `metrics`.

- [ ] Green: implement pure NumPy/SciPy metrics with no ROS node side effects.

```python
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.spatial import cKDTree

from .bag_reader import EvidenceRecord, Pose2D, interpolate_pose


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
    return int(scan.header.stamp.sec) * 1_000_000_000 + int(scan.header.stamp.nanosec)


def _stationary_mask(odom: Sequence[Pose2D], stamps: Sequence[int]) -> list[bool]:
    stationary = []
    for stamp in stamps:
        pose = interpolate_pose(odom, stamp)
        stationary.append(pose is not None)
    return stationary


def compute_scan_metrics(scan_records: Sequence[Any], odom_poses: Sequence[Pose2D]) -> ScanMetrics:
    stamps = [_scan_stamp_ns(s) for s in scan_records]
    duration = (max(stamps) - min(stamps)) / 1e9 if len(stamps) > 1 else 0.0
    intervals = np.diff(sorted(stamps)) / 1e9 if len(stamps) > 1 else np.array([])
    rate = float(1.0 / np.median(intervals)) if intervals.size else 0.0
    rate_cv = float(np.std(intervals) / np.mean(intervals)) if intervals.size and np.mean(intervals) > 0 else 0.0
    beam_count = len(scan_records[0].ranges) if scan_records else 0
    stationary_scans = [s for s, keep in zip(scan_records, _stationary_mask(odom_poses, stamps)) if keep]
    if not stationary_scans:
        return ScanMetrics(duration, rate, rate_cv, beam_count, 0, None)
    arr = np.array([[r if math.isfinite(r) else np.nan for r in s.ranges] for s in stationary_scans], dtype=float)
    finite_ratio = np.mean(np.isfinite(arr), axis=0)
    qualified = finite_ratio >= 0.8
    stable_bins = int(np.sum(qualified))
    mad_p95 = None
    if stable_bins:
        med = np.nanmedian(arr[:, qualified], axis=0)
        mad = np.nanmedian(np.abs(arr[:, qualified] - med), axis=0)
        mad_p95 = float(np.nanpercentile(mad, 95))
    return ScanMetrics(duration, rate, rate_cv, beam_count, stable_bins, mad_p95)


def compute_odom_tf_metrics(odom: Sequence[Pose2D], tf: Sequence[Pose2D]) -> OdomTfMetrics:
    pos_errors = []
    yaw_errors = []
    matched = 0
    for pose in odom:
        tf_pose = interpolate_pose(tf, pose.stamp_ns, max_gap_ns=50_000_000)
        if tf_pose is None:
            continue
        matched += 1
        pos_errors.append(math.hypot(pose.x - tf_pose.x, pose.y - tf_pose.y))
        yaw_errors.append(abs(_wrap_pi(pose.yaw - tf_pose.yaw)))
    jumps_t = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(odom, odom[1:])]
    jumps_y = [abs(_wrap_pi(b.yaw - a.yaw)) for a, b in zip(odom, odom[1:])]
    return OdomTfMetrics(
        matched / len(odom) if odom else 0.0,
        float(np.percentile(pos_errors, 95)) if pos_errors else None,
        float(np.percentile(yaw_errors, 95)) if yaw_errors else None,
        max(jumps_t, default=0.0),
        max(jumps_y, default=0.0),
    )


def _endpoints(scan: Any, pose: Pose2D) -> np.ndarray:
    pts = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        bx, by = r * math.cos(angle), r * math.sin(angle)
        ox = pose.x + math.cos(pose.yaw) * bx - math.sin(pose.yaw) * by
        oy = pose.y + math.sin(pose.yaw) * bx + math.cos(pose.yaw) * by
        pts.append((ox, oy))
    return np.asarray(pts, dtype=float)


def compute_compensated_residuals(scans: Sequence[Any], odom: Sequence[Pose2D]) -> CompensatedResidualMetrics:
    residuals = []
    overlaps = []
    pairs = 0
    for a, b in zip(scans, scans[1:]):
        ta, tb = _scan_stamp_ns(a), _scan_stamp_ns(b)
        if tb - ta > 200_000_000:
            continue
        pa, pb = interpolate_pose(odom, ta), interpolate_pose(odom, tb)
        if pa is None or pb is None:
            continue
        ea, eb = _endpoints(a, pa), _endpoints(b, pb)
        if len(ea) == 0 or len(eb) == 0:
            continue
        dists, _ = cKDTree(ea).query(eb, k=1)
        residuals.extend(float(x) for x in dists)
        overlaps.append(float(np.mean(dists <= 0.20)))
        pairs += 1
    return CompensatedResidualMetrics(
        float(np.percentile(residuals, 95)) if residuals else None,
        float(np.median(overlaps)) if overlaps else None,
        pairs,
    )
```

- [ ] Finish green implementation for timestamp reversals and profile metrics:

```python
def count_timestamp_reversals(records: Sequence[EvidenceRecord]) -> dict[str, int]:
    last: dict[str, int] = {}
    counts: dict[str, int] = {}
    for record in records:
        if record.topic in last and record.source_time_ns < last[record.topic]:
            counts[record.topic] = counts.get(record.topic, 0) + 1
        last[record.topic] = record.source_time_ns
        counts.setdefault(record.topic, 0)
    return counts


def compute_profile_metrics(samples: Sequence[Mapping[str, Any]]) -> dict[str, float | int | bool]:
    cpus = [float(s["total_cpu_percent"]) for s in samples if "total_cpu_percent" in s]
    mem = [float(s["available_memory_bytes"]) for s in samples if "available_memory_bytes" in s]
    disk = [float(s["disk_free_bytes"]) for s in samples if "disk_free_bytes" in s]
    return {
        "total_cpu_p95_pct": float(np.percentile(cpus, 95)) if cpus else math.nan,
        "min_free_memory_gib": min(mem) / 1024 ** 3 if mem else math.nan,
        "min_free_disk_gib": min(disk) / 1024 ** 3 if disk else math.nan,
        "sample_count": len(samples),
        "has_cpu_memory_disk": bool(cpus and mem and disk),
    }
```

- [ ] Register and run:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_metrics.py
colcon test-result --verbose
```

Expected pass names: `test_stationary_scan_mad_and_rate`, `test_yaw_wrap_tf_consistency_and_jumps`, `test_compensated_residual_static_wall_overlap`, `test_timestamp_reversal_and_profile_metrics`.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Task 7: Gate and CLI

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/gate.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/slam_dynamic_filter/cli.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/scripts/sdfctl`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_gate.py`
- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/test_cli.py`

**Interfaces**

```python
# slam_dynamic_filter/gate.py
@dataclass(frozen=True)
class Check:
    name: str
    measured: Any
    operator: str
    threshold: Any
    passed: bool
    reason: str

def load_thresholds(path: pathlib.Path) -> dict[str, Any]: ...
def evaluate_gate(session_id: str, config_hash: str, git_sha: str, metrics: Mapping[str, Any], thresholds: Mapping[str, Any]) -> dict[str, Any]: ...
def write_reports_atomic(session_paths: SessionPaths, metrics: Mapping[str, Any], gate: Mapping[str, Any]) -> None: ...

# slam_dynamic_filter/cli.py
def build_parser() -> argparse.ArgumentParser: ...
def main(argv: Sequence[str] | None = None) -> int: ...
```

**Steps**

- [ ] Red: create gate tests for pass, threshold block, and missing metric fail-closed.

```python
from slam_dynamic_filter.gate import evaluate_gate


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
        "scan_stability": {"duration_sec": 120.0, "rate_hz": 10.0, "rate_cv": 0.02, "beam_count": 723, "stationary_stable_bins": 250, "stationary_range_mad_p95_m": 0.03},
        "same_source_odom_consistency": {"tf_coverage": 1.0, "position_p95_m": 0.02, "yaw_p95_rad": 0.02, "translation_jump_max_m": 0.1, "yaw_jump_max_rad": 0.1},
        "compensated_scan": {"nn_residual_p95_m": 0.08, "overlap_median": 0.9},
        "timestamps": {"max_reversals": 0},
        "r5_profile": {"total_cpu_p95_pct": 55.0, "min_free_memory_gib": 2.0, "min_free_disk_gib": 30.0, "has_cpu_memory_disk": True},
    }


def test_gate_passes_complete_metrics():
    gate = evaluate_gate("20260720T010203Z-static-wall-motion", "a" * 64, "abc123", passing_metrics(), thresholds())
    assert gate["schema_version"] == 1
    assert gate["same_source_odom"] is True
    assert gate["allow_enforce"] is True
    assert gate["blocked_codes"] == []


def test_gate_blocks_threshold_failure():
    metrics = passing_metrics()
    metrics["scan_stability"]["rate_hz"] = 15.0
    gate = evaluate_gate("20260720T010203Z-static-wall-motion", "a" * 64, "abc123", metrics, thresholds())
    assert gate["allow_enforce"] is False
    assert "scan_rate_hz_max" in gate["blocked_codes"]


def test_gate_fail_closed_on_missing_metric():
    metrics = passing_metrics()
    del metrics["r5_profile"]["min_free_disk_gib"]
    gate = evaluate_gate("20260720T010203Z-static-wall-motion", "a" * 64, "abc123", metrics, thresholds())
    assert gate["allow_enforce"] is False
    assert "min_free_disk_gib" in gate["blocked_codes"]
```

- [ ] Red: create CLI parser tests for stable exit codes.

```python
from slam_dynamic_filter.cli import build_parser
from slam_dynamic_filter.constants import EXIT_USAGE


def test_cli_has_doctor_record_analyze():
    parser = build_parser()
    assert parser.parse_args(["doctor"]).command == "doctor"
    assert parser.parse_args(["record", "--duration-sec", "120"]).duration_sec == 120
    assert parser.parse_args(["analyze", "--session", "20260720T010203Z-static-wall-motion"]).command == "analyze"


def test_cli_usage_errors_are_code_2():
    assert EXIT_USAGE == 2
```

- [ ] Run red:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_gate.py test/test_cli.py
```

Expected red result: missing `gate` and `cli`.

- [ ] Green: implement explicit named checks and CLI.

```python
# gate.py
from __future__ import annotations

import math
import pathlib
from dataclasses import asdict, dataclass
from typing import Any, Mapping

import yaml

from .constants import SCHEMA_VERSION
from .metadata import SessionPaths, atomic_write_json


@dataclass(frozen=True)
class Check:
    name: str
    measured: Any
    operator: str
    threshold: Any
    passed: bool
    reason: str


def load_thresholds(path: pathlib.Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("threshold schema_version must be 1")
    return data


def _value(metrics: Mapping[str, Any], dotted: str) -> Any:
    cur: Any = metrics
    for part in dotted.split("."):
        if not isinstance(cur, Mapping) or part not in cur:
            return None
        cur = cur[part]
    return cur


def _check(metrics: Mapping[str, Any], name: str, path: str, op: str, threshold: Any) -> Check:
    measured = _value(metrics, path)
    if measured is None or (isinstance(measured, float) and math.isnan(measured)):
        return Check(name, measured, op, threshold, False, "missing or unmeasured")
    passed = {
        ">=": measured >= threshold,
        "<=": measured <= threshold,
        "==": measured == threshold,
    }[op]
    return Check(name, measured, op, threshold, bool(passed), "ok" if passed else "threshold failed")


def evaluate_gate(session_id: str, config_hash: str, git_sha: str, metrics: Mapping[str, Any], thresholds: Mapping[str, Any]) -> dict[str, Any]:
    specs = [
        ("min_duration_sec", "scan_stability.duration_sec", ">=", thresholds["min_duration_sec"]),
        ("scan_rate_hz_min", "scan_stability.rate_hz", ">=", thresholds["scan_rate_hz_min"]),
        ("scan_rate_hz_max", "scan_stability.rate_hz", "<=", thresholds["scan_rate_hz_max"]),
        ("scan_rate_cv_max", "scan_stability.rate_cv", "<=", thresholds["scan_rate_cv_max"]),
        ("expected_beam_count", "scan_stability.beam_count", "==", thresholds["expected_beam_count"]),
        ("min_stationary_stable_bins", "scan_stability.stationary_stable_bins", ">=", thresholds["min_stationary_stable_bins"]),
        ("stationary_range_mad_p95_max_m", "scan_stability.stationary_range_mad_p95_m", "<=", thresholds["stationary_range_mad_p95_max_m"]),
        ("tf_coverage_min", "same_source_odom_consistency.tf_coverage", ">=", thresholds["tf_coverage_min"]),
        ("odom_tf_position_p95_max_m", "same_source_odom_consistency.position_p95_m", "<=", thresholds["odom_tf_position_p95_max_m"]),
        ("odom_tf_yaw_p95_max_rad", "same_source_odom_consistency.yaw_p95_rad", "<=", thresholds["odom_tf_yaw_p95_max_rad"]),
        ("odom_translation_jump_max_m", "same_source_odom_consistency.translation_jump_max_m", "<=", thresholds["odom_translation_jump_max_m"]),
        ("odom_yaw_jump_max_rad", "same_source_odom_consistency.yaw_jump_max_rad", "<=", thresholds["odom_yaw_jump_max_rad"]),
        ("compensated_nn_residual_p95_max_m", "compensated_scan.nn_residual_p95_m", "<=", thresholds["compensated_nn_residual_p95_max_m"]),
        ("compensated_overlap_min", "compensated_scan.overlap_median", ">=", thresholds["compensated_overlap_min"]),
        ("timestamp_reversal_max", "timestamps.max_reversals", "<=", thresholds["timestamp_reversal_max"]),
        ("total_cpu_p95_max_pct", "r5_profile.total_cpu_p95_pct", "<=", thresholds["total_cpu_p95_max_pct"]),
        ("min_free_memory_gib", "r5_profile.min_free_memory_gib", ">=", thresholds["min_free_memory_gib"]),
        ("min_free_disk_gib", "r5_profile.min_free_disk_gib", ">=", thresholds["min_free_disk_gib"]),
    ]
    checks = [_check(metrics, *spec) for spec in specs]
    blocked = [c.name for c in checks if not c.passed]
    return {
        "schema_version": SCHEMA_VERSION,
        "session_id": session_id,
        "config_hash": config_hash,
        "git_sha": git_sha,
        "same_source_odom": True,
        "scan_stability": metrics.get("scan_stability"),
        "same_source_odom_consistency": metrics.get("same_source_odom_consistency"),
        "r5_profile": metrics.get("r5_profile"),
        "checks": [asdict(c) for c in checks],
        "allow_enforce": not blocked,
        "blocked_codes": blocked,
    }


def write_reports_atomic(session_paths: SessionPaths, metrics: Mapping[str, Any], gate: Mapping[str, Any]) -> None:
    atomic_write_json(session_paths.reports / "stage0_metrics.json", metrics)
    atomic_write_json(session_paths.c1_gate_json, gate)
```

```python
# cli.py
from __future__ import annotations

import argparse
from typing import Sequence

from .constants import EXIT_SUCCESS


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="sdfctl")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("doctor")
    record = sub.add_parser("record")
    record.add_argument("--duration-sec", type=int, default=120)
    record.add_argument("--include-scan-baseline", action="store_true")
    analyze = sub.add_parser("analyze")
    analyze.add_argument("--session", required=True)
    analyze.add_argument("--thresholds", default=None)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command in {"doctor", "record", "analyze"}:
        return EXIT_SUCCESS
    return EXIT_SUCCESS
```

```python
#!/usr/bin/env python3
from slam_dynamic_filter.cli import main

raise SystemExit(main())
```

- [ ] Complete CLI wiring before green is accepted: `doctor` calls preflight and exits `0` or `3`; `record` validates duration, runs preflight, then `record_session` and exits `0`, `3`, or `4`; `analyze` validates metadata/manifest/config hash, reads bag, computes metrics, evaluates gate, writes atomic reports, exits `0` for pass, `6` for valid threshold block, `5` for invalid evidence, `7` only for unexpected internal errors. Parser usage errors return `2`.
- [ ] Register and run:

```bash
colcon test --packages-select slam_dynamic_filter --pytest-args -q test/test_gate.py test/test_cli.py
colcon test-result --verbose
```

Expected pass names: `test_gate_passes_complete_metrics`, `test_gate_blocks_threshold_failure`, `test_gate_fail_closed_on_missing_metric`, `test_cli_has_doctor_record_analyze`, `test_cli_usage_errors_are_code_2`.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Task 8: Integration, Docs, and On-Vehicle Evidence Procedure

**Files**

- `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/README.md`
- `cod_-rm2026_-navigation-master/docs/MAPPING_EVIDENCE_RUNBOOK.md`
- `cod_-rm2026_-navigation-master/docs/QUICK_START.md`
- All package files from Tasks 1-7 for final registration.

**Interfaces**

```text
sdfctl doctor
sdfctl record --duration-sec 120
sdfctl analyze --session <session_id>
ros2 launch slam_dynamic_filter evidence_scan.launch.py
```

**Steps**

- [ ] Red: run all tests before docs and integration registration are complete.

```bash
colcon test --packages-select slam_dynamic_filter
colcon test-result --verbose
```

Expected red result at this point if registration is incomplete: one or more missing pytest registrations or CLI wrapper install failures.

- [ ] Green: complete `CMakeLists.txt` test registration for all tests.

```cmake
ament_add_pytest_test(test_constants test/test_constants.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_metadata test/test_metadata.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_preflight test/test_preflight.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_recorder test/test_recorder.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_bag_reader test/test_bag_reader.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_metrics test/test_metrics.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_gate test/test_gate.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_cli test/test_cli.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
ament_add_pytest_test(test_evidence_scan_launch test/test_evidence_scan_launch.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] Add package `README.md` with exact commands and Stage 0 limitation:

```markdown
# slam_dynamic_filter

Stage 0 evidence package. It records and analyzes `/scan_raw`, `/Odometry`, `/tf`, `/tf_static`, Livox point clouds, and Livox IMU for C1 gating.

It does not publish a tracker, mask `/scan`, remap slam_toolbox, save maps, replace autosave, or enable enforce mode.

## Commands

```bash
ros2 launch slam_dynamic_filter evidence_scan.launch.py
sdfctl doctor
sdfctl record --duration-sec 120
sdfctl analyze --session <session_id>
```
```

- [ ] Add `docs/MAPPING_EVIDENCE_RUNBOOK.md` operator sequence:

```markdown
# Mapping Evidence Runbook

## Start

```bash
cd /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
source install/setup.bash
ros2 launch slam_dynamic_filter evidence_scan.launch.py
```

In another terminal:

```bash
source install/setup.bash
sdfctl doctor
ros2 topic hz /scan_raw
ros2 topic info /scan_raw
sdfctl record --duration-sec 120
sdfctl analyze --session <session_id>
```

Expected `/scan_raw` type: `sensor_msgs/msg/LaserScan`. Expected approximate rate: `8-12 Hz`.

## 120-Second Scenario

- 0-20 s stationary facing a textured static wall, no people/robots in 8 m measurement zone.
- 20-40 s slow yaw left then right, <=0.4 rad/s, keep same wall visible.
- 40-60 s slow forward then backward, <=0.3 m/s, keep >=1 m from wall.
- 60-80 s slow lateral left then right, <=0.3 m/s.
- 80-100 s combined low-speed translation/yaw, limits above.
- 100-120 s stationary at approximately starting pose; no dynamic objects.

C1 BLOCKED is valid when evidence is complete but thresholds fail. Do not tune thresholds around a failing robot result.
```

- [ ] Modify only `docs/QUICK_START.md` with one link line:

```markdown
For Stage 0 SLAM dynamic-filter evidence collection, see [MAPPING_EVIDENCE_RUNBOOK.md](MAPPING_EVIDENCE_RUNBOOK.md).
```

- [ ] Build and test:

```bash
colcon build --packages-select slam_dynamic_filter --symlink-install
source install/setup.bash
colcon test --packages-select slam_dynamic_filter
colcon test-result --verbose
```

Expected result: all nine test files pass.

- [ ] Launch and topic checks on robot:

```bash
source install/setup.bash
ros2 launch slam_dynamic_filter evidence_scan.launch.py
ros2 topic info /scan_raw
ros2 topic hz /scan_raw
```

Expected PASS line from `sdfctl doctor`:

```text
PASS preflight: required topics and disk available
```

Expected valid blocked line when metrics fail thresholds:

```text
BLOCKED C1: valid evidence but thresholds failed
```

Expected pass line only when all checks pass:

```text
PASS C1: evidence gate passed
```

- [ ] Validate artifact layout after a real run:

```bash
find "$HOME/cod_mapping_sessions/<session_id>" -maxdepth 2 -type f | sort
python3 -m json.tool "$HOME/cod_mapping_sessions/<session_id>/evidence/c1_gate.json" >/dev/null
python3 -m json.tool "$HOME/cod_mapping_sessions/<session_id>/reports/stage0_metrics.json" >/dev/null
```

Expected files include `metadata.json`, `record_manifest.json`, `system_profile.jsonl`, `reports/stage0_metrics.json`, and `evidence/c1_gate.json`.

- [ ] Rollback/cleanup procedure if operator aborts before 120 s:

```bash
rm -rf "$HOME/cod_mapping_sessions/<aborted_session_id>"
```

Do not delete successful evidence sessions. Actual hardware C1 cannot be marked complete until the 120-second robot scenario is captured and analyzed. `C1 BLOCKED` is a valid test outcome and must not be tuned around.

- [ ] Review checkpoint:

```bash
git diff --check
git status --short
```

## Verification Matrix

- AMD Ryzen 5 4500U, 6c/6t, no GPU: Task 6 `compute_profile_metrics`, Task 8 runbook, gate checks `total_cpu_p95_max_pct`, memory, disk.
- MID-360 built-in IMU only; no electrical IMU/wheel/Realsense: Task 4 required topics and runbook; no extra sensor topics accepted as required.
- `/Odometry` same-source evidence, not ground truth: Task 5 pose normalization, Task 6 odom/TF consistency name, Task 7 `same_source_odom=true`.
- No modification to production `/scan`, slam_toolbox, Nav2, autosave, `multiplenav_launch.py`, mapper params: Task 3 launch test, Task 8 docs-only QUICK_START edit.
- `/scan_raw` projector exact config: Task 3 `test_evidence_scan_parameters_are_locked`.
- Required bag topics and types: Task 1 constants, Task 4 preflight, Task 5 bag validation.
- Recorder defaults and storage: Task 4 `build_rosbag_argv`, duration bounds, disk preflight.
- Direct subprocess arrays only: Task 4 `test_rosbag_argv_is_direct_and_locked`.
- JSON schema/stable formatting: Task 1 schema, Task 2 `atomic_write_json`, Task 7 gate.
- Fail-closed gate: Task 7 `test_gate_fail_closed_on_missing_metric`.
- Evidence-only, no enforce runtime mode: Task 7 gate field only, Task 8 README limitation.
- Existing 7.84 s CustomMsg bag invalid: Task 4 type mismatch CustomMsg rejects, Task 7 duration threshold.
- TDD red before green: every task begins with a red test command and named expected red result.
- No unauthorized commit/push: every task ends with `git diff --check` and `git status --short`; no commit command appears.
- Beam count 723 from converter `ceil`: Task 1 characterization test.
- 120-second scenario and odom-derived stationary detection: Task 4 scenario prompts, Task 6 stationary metrics.
- Deterministic ordering and duplicate timestamps: Task 5 `test_sort_records_uses_source_time_priority_sequence_and_preserves_duplicates`.
- SE(2), yaw wrap, TF selection: Task 5 interpolation, Task 6 yaw wrap consistency.
- Stationary MAD and compensated cKDTree residual: Task 6 scan and residual tests.
- Timestamp reversal and bag timestamp fallback: Task 5 source timestamp, Task 6 reversal count.
- System sidecar required fields and thermal non-blocking: Task 4 system sample, Task 6 profile metrics.
- Atomic analyze outputs: Task 2 atomic JSON, Task 7 `write_reports_atomic`.

## Explicit Non-Goals

- Tracker.
- Session dynamic memory.
- Masking.
- `/scan_slam_filtered`.
- SLAM remap.
- Safe saver.
- Autosave replacement.
- Offline refinement.
- Map promotion.
- Nav2 changes.
- Collision behavior changes.

## Hardware Gate

Implementation can be locally complete when the package builds, all tests pass, launch configuration is verified, CLI commands have stable exit codes, and docs are linked. Phase 17/C1 remains pending until the user runs the full 120-second robot scenario and analyzes the resulting session under `~/cod_mapping_sessions/<session_id>`.

`C1 BLOCKED` is a valid evidence outcome when the bag is complete but thresholds fail. Do not silently tune `stage0_thresholds.yaml` around a blocked result; threshold changes require an explicit reviewed plan update.

## Plan Self-Review

- [x] Spec coverage: every requested file, constraint, threshold, algorithm, artifact path, and exit code is represented.
- [x] Placeholder scan: all steps contain concrete files, commands, snippets, and exact behavior.
- [x] Type consistency: required ROS topic names and exact message types match the constraint block.
- [x] Source-path safety: Stage 0 leaves production launch/config paths untouched except the allowed `docs/QUICK_START.md` link during implementation.
- [x] No unauthorized commits: plan contains review checkpoints only and no `git commit` or push command.

## Execution Handoff

Subagent-Driven option: use `superpowers:subagent-driven-development` to assign independent implementation slices, but every subagent must follow the red/green/review order above and must not touch files outside the locked structure.

Inline Execution option: use `superpowers:executing-plans` and execute Tasks 1-8 directly in order. Project `CLAUDE.md` defaults main-agent direct execution, so Inline Execution is recommended unless the user explicitly wants delegation.

Before implementation, use `superpowers:using-git-worktrees` because project instructions require isolated development. Then run each task from `/home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master`, preserve the no-commit rule, and leave hardware C1 pending until a real 120-second evidence session exists.
