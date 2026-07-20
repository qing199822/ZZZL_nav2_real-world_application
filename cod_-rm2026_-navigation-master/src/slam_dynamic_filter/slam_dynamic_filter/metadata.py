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
    try:
        common = os.path.commonpath((str(root_real), str(child_real)))
    except ValueError as exc:
        raise ValueError(f"path outside output root: {child}") from exc
    if common != str(root_real):
        raise ValueError(f"path outside output root: {child}")
    return child_real


def create_session_paths(output_root: pathlib.Path, session_id: str) -> SessionPaths:
    if not SESSION_ID_RE.fullmatch(session_id):
        raise ValueError(f"invalid session_id: {session_id}")
    output_root = output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    root = canonical_child(output_root, output_root / session_id)
    root.mkdir(mode=0o750, exist_ok=False)
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
    for directory in (paths.config, paths.logs, paths.reports, paths.evidence):
        directory.mkdir(mode=0o750)
    return paths


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(path: pathlib.Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(
                payload,
                stream,
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, path)
        dir_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
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


def frozen_metadata(
    session_id: str,
    scenario: str,
    git_sha: str,
    config_hash: str,
) -> dict[str, Any]:
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
    session_id = manifest.get("session_id")
    if not isinstance(session_id, str) or not SESSION_ID_RE.fullmatch(session_id):
        errors.append("session_id invalid")
    if manifest.get("scenario") != SCENARIO_ID:
        errors.append("scenario invalid")
    if not isinstance(manifest.get("git_sha"), str) or not manifest["git_sha"]:
        errors.append("git_sha missing")
    config_hash = manifest.get("config_hash")
    if (
        not isinstance(config_hash, str)
        or len(config_hash) != 64
        or any(character not in "0123456789abcdef" for character in config_hash)
    ):
        errors.append("config_hash invalid")
    if manifest.get("same_source_odom") is not True:
        errors.append("same_source_odom must be true")
    return errors
