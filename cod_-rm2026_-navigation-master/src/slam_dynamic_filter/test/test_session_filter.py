import math

import pytest

from slam_dynamic_filter.bag_reader import Pose2D
from slam_dynamic_filter.session_filter import (
    FilterConfig,
    FilterInputError,
    ScanSample,
    SessionFilter,
)


def scan(stamp_ns, object_start=None, object_width=6, object_range=2.0, wall=False):
    ranges = [float("inf")] * 181
    for index in range(20, 45):
        ranges[index] = 5.0
    if wall:
        for index in range(55, 126):
            ranges[index] = object_range
    elif object_start is not None:
        for index in range(object_start, object_start + object_width):
            ranges[index] = object_range
    return ScanSample(
        stamp_ns=stamp_ns,
        angle_min=-1.8,
        angle_increment=0.02,
        range_min=0.5,
        range_max=20.0,
        ranges=tuple(ranges),
    )


def config(**overrides):
    values = {
        "buffer_frames": 3,
        "segment_gap_m": 0.30,
        "min_segment_points": 3,
        "min_segment_diameter_m": 0.03,
        "max_candidate_diameter_m": 1.0,
        "association_distance_m": 0.75,
        "dynamic_reacquire_distance_m": 0.85,
        "motion_displacement_m": 0.15,
        "min_motion_speed_mps": 0.10,
        "motion_confirmation_frames": 3,
        "motion_consistency_min": 0.70,
        "max_missed_frames": 4,
        "max_tracks": 32,
    }
    values.update(overrides)
    return FilterConfig(**values)


def object_mask_count(frame):
    return sum(math.isnan(value) for value in frame.ranges)


def test_moved_object_stays_dynamic_after_it_stops():
    engine = SessionFilter(config())
    outputs = []
    starts = [80, 83, 86, 89] + [89] * 12
    for frame_index, start in enumerate(starts):
        result = engine.process_scan(
            scan(frame_index * 100_000_000, object_start=start),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )
        if result is not None:
            outputs.append(result)

    assert engine.dynamic_track_count == 1
    assert object_mask_count(outputs[-1]) == 6
    assert all(
        math.isnan(value)
        for value in outputs[-1].ranges[89:95]
    )


def test_ego_motion_is_compensated_in_odom_frame():
    engine = SessionFilter(config())
    outputs = []
    for frame_index in range(12):
        robot_x = frame_index * 0.05
        result = engine.process_scan(
            scan(
                frame_index * 100_000_000,
                object_start=88,
                object_range=3.0 - robot_x,
            ),
            Pose2D(frame_index * 100_000_000, robot_x, 0.0, 0.0),
        )
        if result is not None:
            outputs.append(result)

    assert engine.dynamic_track_count == 0
    assert all(object_mask_count(output) == 0 for output in outputs)


def test_long_wall_is_not_a_motion_candidate_when_centroid_shifts():
    engine = SessionFilter(config(max_candidate_diameter_m=0.8))
    for frame_index in range(10):
        result = engine.process_scan(
            scan(
                frame_index * 100_000_000,
                object_range=2.0 + 0.04 * frame_index,
                wall=True,
            ),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )
        if result is not None:
            assert object_mask_count(result) == 0

    assert engine.dynamic_track_count == 0


def test_oversized_static_segments_do_not_consume_track_capacity():
    engine = SessionFilter(config(max_candidate_diameter_m=0.8, max_tracks=1))

    engine.process_scan(
        scan(0, wall=True),
        Pose2D(0, 0.0, 0.0, 0.0),
    )

    assert engine.track_count == 0
    assert engine.capacity_exceeded is False


def test_dynamic_track_does_not_mask_a_large_segment_after_wall_merge():
    engine = SessionFilter(config(max_candidate_diameter_m=0.8))
    output = None
    sequence = [80, 83, 86, 89]
    for frame_index, start in enumerate(sequence):
        engine.process_scan(
            scan(frame_index * 100_000_000, object_start=start),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )
    for frame_index in range(4, 10):
        output = engine.process_scan(
            scan(frame_index * 100_000_000, wall=True),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )

    assert engine.dynamic_track_count == 1
    assert output is not None
    assert output.dynamic_beam_count == 0


def test_dynamic_identity_survives_short_occlusion_and_nearby_reappearance():
    engine = SessionFilter(config())
    sequence = [80, 83, 86, 89, None, None, 89, 89, 89, 89, 89]
    outputs = []
    for frame_index, start in enumerate(sequence):
        result = engine.process_scan(
            scan(frame_index * 100_000_000, object_start=start),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )
        if result is not None:
            outputs.append(result)

    assert engine.dynamic_track_count == 1
    assert any(output.dynamic_beam_count == 6 for output in outputs[-3:])


def test_mask_uses_nan_and_preserves_real_no_return_inf():
    engine = SessionFilter(config())
    output = None
    for frame_index, start in enumerate([80, 83, 86, 89, 89, 89, 89, 89]):
        output = engine.process_scan(
            scan(frame_index * 100_000_000, object_start=start),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )

    assert output is not None
    assert output.dynamic_beam_count == 6
    assert all(math.isnan(value) for value in output.ranges[89:95])
    assert math.isinf(output.ranges[0]) and output.ranges[0] > 0.0


def test_timestamp_reversal_is_fail_closed_without_mutating_tracks():
    engine = SessionFilter(config())
    engine.process_scan(scan(100, object_start=80), Pose2D(100, 0.0, 0.0, 0.0))
    track_count = engine.track_count

    with pytest.raises(FilterInputError, match="timestamp"):
        engine.process_scan(scan(99, object_start=80), Pose2D(99, 0.0, 0.0, 0.0))

    assert engine.track_count == track_count


def test_implausible_centroid_step_speed_does_not_confirm_motion():
    engine = SessionFilter(config(max_confirmation_step_speed_mps=1.2))
    for frame_index, start in enumerate([80, 90, 100, 110]):
        engine.process_scan(
            scan(frame_index * 100_000_000, object_start=start),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )

    assert engine.dynamic_track_count == 0


def test_gapped_observations_do_not_confirm_continuous_motion():
    engine = SessionFilter(config(max_confirmation_gap_sec=0.15))
    for frame_index, start in enumerate([80, None, 83, None, 86, None, 89]):
        engine.process_scan(
            scan(frame_index * 100_000_000, object_start=start),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )

    assert engine.dynamic_track_count == 0


def test_reset_clears_session_dynamic_memory_and_buffer():
    engine = SessionFilter(config())
    for frame_index, start in enumerate([80, 83, 86, 89]):
        engine.process_scan(
            scan(frame_index * 100_000_000, object_start=start),
            Pose2D(frame_index * 100_000_000, 0.0, 0.0, 0.0),
        )
    assert engine.dynamic_track_count == 1

    engine.reset()

    assert engine.dynamic_track_count == 0
    assert engine.track_count == 0
    assert engine.buffered_frame_count == 0
