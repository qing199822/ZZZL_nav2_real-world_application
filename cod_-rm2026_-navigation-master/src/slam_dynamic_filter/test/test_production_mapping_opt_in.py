import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def _load_launch():
    path = Path(__file__).parents[2] / "cod_bringup" / "launch" / "multiplenav_launch.py"
    spec = importlib.util.spec_from_file_location("multiplenav_launch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description(), path.read_text(encoding="utf-8")


def _text(value):
    if isinstance(value, (tuple, list)):
        return perform_substitutions(LaunchContext(), list(value))
    return value


def _walk(entities):
    for entity in entities:
        yield entity
        if isinstance(entity, GroupAction):
            actions = getattr(entity, "actions", None)
            if actions is None:
                actions = vars(entity)["_GroupAction__actions"]
            yield from _walk(actions)


def test_production_mapping_filter_is_explicit_and_disabled_by_default():
    description, source = _load_launch()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    mode = arguments["slam_filter_mode"]
    assert _text(mode.default_value) == "disabled"
    assert set(mode.choices) == {"disabled", "observe", "enforce"}
    assert "mapping_filter.launch.py" in source


def test_filtered_mapping_has_conditional_slam_route_and_no_autosave():
    description, source = _load_launch()
    entities = list(_walk(description.entities))
    slam_nodes = [
        entity
        for entity in entities
        if isinstance(entity, Node) and entity.node_package == "slam_toolbox"
    ]
    assert len(slam_nodes) == 2
    assert all(node.condition is not None for node in slam_nodes)

    includes = [entity for entity in entities if isinstance(entity, IncludeLaunchDescription)]
    assert any(include.condition is not None for include in includes)
    assert "('/scan', '/scan_slam_filtered')" in source
    assert "filter_disabled_condition" in source
