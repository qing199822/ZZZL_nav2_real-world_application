import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import yaml


def _load_launch():
    path = Path(__file__).parents[1] / "launch" / "evidence_scan.launch.py"
    spec = importlib.util.spec_from_file_location("evidence_scan_launch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _text(value):
    if isinstance(value, (tuple, list)):
        return perform_substitutions(LaunchContext(), list(value))
    return value


def _remappings(node):
    raw = getattr(node, "node_remappings", None)
    if raw is None:
        raw = vars(node)["_Node__remappings"]
    return [(_text(source), _text(target)) for source, target in raw]


def _parameters(node):
    raw = getattr(node, "node_parameters", None)
    if raw is None:
        raw = vars(node)["_Node__parameters"]
    normalized = {}
    for key, value in raw[0].items():
        decoded = _text(value)
        normalized[_text(key)] = yaml.safe_load(decoded) if isinstance(decoded, str) else decoded
    return normalized


def test_evidence_scan_projects_filtered_cloud_to_scan_raw_only():
    nodes = [entity for entity in _load_launch().entities if isinstance(entity, Node)]
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == "pointcloud_to_laserscan"
    assert node.node_executable == "pointcloud_to_laserscan_node"
    remappings = _remappings(node)
    assert ("cloud_in", "/livox/lidar_filtered") in remappings
    assert ("scan", "/scan_raw") in remappings
    assert all(target != "/scan" for _, target in remappings)


def test_evidence_scan_parameters_are_locked():
    node = [entity for entity in _load_launch().entities if isinstance(entity, Node)][0]
    params = _parameters(node)
    assert params == {
        "target_frame": "base_link",
        "transform_tolerance": 0.01,
        "min_height": 0.05,
        "max_height": 1.0,
        "angle_min": -3.1416,
        "angle_max": 3.1416,
        "angle_increment": 0.0087,
        "scan_time": 0.1,
        "range_min": 0.5,
        "range_max": 20.0,
        "use_inf": True,
    }
