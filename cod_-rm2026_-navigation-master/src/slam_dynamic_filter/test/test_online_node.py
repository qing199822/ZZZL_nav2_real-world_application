import math

from sensor_msgs.msg import LaserScan

from slam_dynamic_filter.online_node import build_output_scans, filter_output_allowed
from slam_dynamic_filter.session_filter import FilteredFrame, ScanSample


def source_scan():
    message = LaserScan()
    message.header.frame_id = "base_link"
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 34
    message.angle_min = -0.1
    message.angle_max = 0.2
    message.angle_increment = 0.1
    message.range_min = 0.5
    message.range_max = 20.0
    message.ranges = [1.0, 2.0, float("inf"), 4.0]
    message.intensities = [10.0, 20.0, 30.0, 40.0]
    return message


def filtered_frame():
    sample = ScanSample(
        stamp_ns=12_000_000_034,
        angle_min=-0.1,
        angle_increment=0.1,
        range_min=0.5,
        range_max=20.0,
        ranges=(1.0, 2.0, float("inf"), 4.0),
    )
    return FilteredFrame(
        sample=sample,
        ranges=(1.0, float("nan"), float("inf"), 4.0),
        dynamic_mask=(False, True, False, False),
        dynamic_beam_count=1,
        active_track_count=1,
        dynamic_track_count=1,
    )


def test_enforce_output_masks_only_dynamic_ranges_and_preserves_metadata():
    output, mask = build_output_scans(source_scan(), filtered_frame(), "enforce")

    assert output.header.frame_id == "base_link"
    assert output.header.stamp.sec == 12
    assert list(output.intensities) == [10.0, 20.0, 30.0, 40.0]
    assert output.ranges[0] == 1.0
    assert math.isnan(output.ranges[1])
    assert math.isinf(output.ranges[2])
    assert math.isnan(mask.ranges[0])
    assert mask.ranges[1] == 2.0
    assert math.isnan(mask.ranges[2])


def test_observe_output_is_unchanged_but_mask_is_still_visible():
    output, mask = build_output_scans(source_scan(), filtered_frame(), "observe")

    assert list(output.ranges) == [1.0, 2.0, float("inf"), 4.0]
    assert mask.ranges[1] == 2.0


def test_unknown_mode_is_rejected():
    try:
        build_output_scans(source_scan(), filtered_frame(), "disabled")
    except ValueError as error:
        assert "mode" in str(error)
    else:
        raise AssertionError("invalid mode was accepted")


def test_unhealthy_or_frozen_filter_never_publishes_to_slam():
    assert filter_output_allowed(frozen=False, capacity_exceeded=False) is True
    assert filter_output_allowed(frozen=True, capacity_exceeded=False) is False
    assert filter_output_allowed(frozen=False, capacity_exceeded=True) is False
