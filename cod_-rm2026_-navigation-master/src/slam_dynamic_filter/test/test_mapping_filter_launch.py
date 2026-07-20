import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def _load_launch():
    path = Path(__file__).parents[1] / "launch" / "mapping_filter.launch.py"
    spec = importlib.util.spec_from_file_location("mapping_filter_launch", path)
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


def test_mapping_filter_launch_defaults_to_observe_and_keeps_scan_separate():
    launch_description = _load_launch()
    arguments = {
        entity.name: entity.default_value
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert _text(arguments["mode"]) == "observe"

    nodes = [entity for entity in launch_description.entities if isinstance(entity, Node)]
    assert len(nodes) == 2
    projector = next(node for node in nodes if node.node_package == "pointcloud_to_laserscan")
    filter_node = next(node for node in nodes if node.node_package == "slam_dynamic_filter")

    assert ("cloud_in", "/livox/lidar_filtered") in _remappings(projector)
    assert ("scan", "/scan_raw") in _remappings(projector)
    assert all(target != "/scan" for _, target in _remappings(projector))
    assert filter_node.node_executable == "slam_dynamic_filter_node"
    assert ("scan_raw", "/scan_raw") in _remappings(filter_node)
    assert ("scan_slam_filtered", "/scan_slam_filtered") in _remappings(filter_node)


def test_pragmatic_config_exposes_all_core_thresholds():
    import yaml

    path = Path(__file__).parents[1] / "config" / "pragmatic_filter.yaml"
    values = yaml.safe_load(path.read_text(encoding="utf-8"))[
        "slam_dynamic_filter_node"
    ]["ros__parameters"]
    assert values["buffer_frames"] == 10
    assert values["motion_displacement_m"] == 0.30
    assert values["min_motion_speed_mps"] == 0.12
    assert values["max_confirmation_step_speed_mps"] == 1.20
    assert values["max_confirmation_gap_sec"] == 0.15
    assert values["max_tracks"] == 32
    assert values["mode"] == "observe"
