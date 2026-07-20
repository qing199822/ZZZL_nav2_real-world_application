import json
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


def test_session_paths_leave_rosbag_output_available_and_json_is_stable(tmp_path):
    paths = create_session_paths(tmp_path, "20260720T010203Z-static-wall-motion")
    assert paths.c1_gate_json == paths.evidence / "c1_gate.json"
    assert not paths.bag.exists()
    atomic_write_json(paths.metadata_json, {"b": 2, "a": 1})
    assert paths.metadata_json.read_text(encoding="utf-8") == '{\n  "a": 1,\n  "b": 2\n}\n'
    assert load_json(paths.metadata_json) == {"a": 1, "b": 2}


def test_sha256_and_manifest_validation():
    fixture = pathlib.Path(__file__).parent / "data" / "abc.txt"
    expected_hash = "edeaaff3f1774ad2888673770c6d64097e391bc362d7d6fb34982ddf0efd18cb"
    assert sha256_file(fixture) == expected_hash
    metadata = frozen_metadata(
        "20260720T010203Z-static-wall-motion",
        "static-wall-motion",
        "abc123",
        "f" * 64,
    )
    assert validate_manifest(metadata) == []
    invalid_path = pathlib.Path(__file__).parent / "data" / "invalid_manifest.json"
    invalid = json.loads(invalid_path.read_text(encoding="utf-8"))
    errors = " ".join(validate_manifest(invalid))
    assert "schema_version" in errors
    assert "session_id" in errors


def test_load_json_rejects_non_object(tmp_path):
    path = tmp_path / "array.json"
    path.write_text("[]\n", encoding="utf-8")
    with pytest.raises(ValueError, match="root must be object"):
        load_json(path)
