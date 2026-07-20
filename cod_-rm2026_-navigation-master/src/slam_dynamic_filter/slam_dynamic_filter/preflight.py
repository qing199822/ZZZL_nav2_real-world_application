from __future__ import annotations

import pathlib
import re
import shutil
import subprocess
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

from .constants import REQUIRED_TOPIC_TYPES

CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess[str]]
DiskProvider = Callable[[pathlib.Path], object]
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


def _disk_probe(path: pathlib.Path) -> pathlib.Path:
    candidate = path.resolve(strict=False)
    while not candidate.exists() and candidate != candidate.parent:
        candidate = candidate.parent
    return candidate


def check_preflight(
    output_root: pathlib.Path,
    run: CommandRunner,
    disk_usage: DiskProvider = shutil.disk_usage,
) -> PreflightResult:
    errors: list[str] = []
    try:
        topic_types = query_topic_types(run)
    except (OSError, RuntimeError) as exc:
        topic_types = {}
        errors.append(f"ROS topic query failed: {exc}")

    usage = disk_usage(_disk_probe(output_root))
    free_bytes = int(usage.free)
    if topic_types:
        for topic, expected in REQUIRED_TOPIC_TYPES.items():
            actual = topic_types.get(topic)
            if actual is None:
                errors.append(f"missing topic {topic}")
            elif actual != expected:
                errors.append(f"type mismatch {topic}: expected {expected}, got {actual}")
    if free_bytes < MIN_FREE_DISK_BYTES:
        errors.append(f"free disk below 20 GiB: {free_bytes}")
    return PreflightResult(not errors, tuple(errors), topic_types, free_bytes)
