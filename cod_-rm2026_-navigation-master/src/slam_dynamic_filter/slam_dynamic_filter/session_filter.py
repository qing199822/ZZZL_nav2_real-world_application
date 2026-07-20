from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence

from .bag_reader import Pose2D


class FilterInputError(ValueError):
    pass


@dataclass(frozen=True)
class FilterConfig:
    buffer_frames: int = 10
    segment_gap_m: float = 0.30
    min_segment_points: int = 3
    min_segment_diameter_m: float = 0.18
    max_candidate_diameter_m: float = 1.50
    association_distance_m: float = 0.75
    dynamic_reacquire_distance_m: float = 0.85
    motion_displacement_m: float = 0.30
    min_motion_speed_mps: float = 0.12
    max_confirmation_step_speed_mps: float = 1.20
    max_confirmation_gap_sec: float = 0.15
    motion_confirmation_frames: int = 10
    motion_consistency_min: float = 0.90
    motion_history_frames: int = 12
    max_missed_frames: int = 4
    max_tracks: int = 32
    min_valid_beams: int = 3
    max_pose_time_delta_sec: float = 0.15
    odom_translation_jump_m: float = 0.50
    odom_yaw_jump_rad: float = 0.50

    def __post_init__(self) -> None:
        positive_values = {
            "buffer_frames": self.buffer_frames,
            "segment_gap_m": self.segment_gap_m,
            "min_segment_points": self.min_segment_points,
            "max_candidate_diameter_m": self.max_candidate_diameter_m,
            "association_distance_m": self.association_distance_m,
            "dynamic_reacquire_distance_m": self.dynamic_reacquire_distance_m,
            "motion_displacement_m": self.motion_displacement_m,
            "min_motion_speed_mps": self.min_motion_speed_mps,
            "max_confirmation_step_speed_mps": self.max_confirmation_step_speed_mps,
            "max_confirmation_gap_sec": self.max_confirmation_gap_sec,
            "motion_confirmation_frames": self.motion_confirmation_frames,
            "motion_history_frames": self.motion_history_frames,
            "max_missed_frames": self.max_missed_frames,
            "max_tracks": self.max_tracks,
            "min_valid_beams": self.min_valid_beams,
            "max_pose_time_delta_sec": self.max_pose_time_delta_sec,
            "odom_translation_jump_m": self.odom_translation_jump_m,
            "odom_yaw_jump_rad": self.odom_yaw_jump_rad,
        }
        invalid = [name for name, value in positive_values.items() if value <= 0]
        if invalid:
            raise ValueError(f"filter parameters must be positive: {', '.join(invalid)}")
        if self.min_segment_diameter_m < 0.0:
            raise ValueError("min_segment_diameter_m must be non-negative")
        if not 0.0 <= self.motion_consistency_min <= 1.0:
            raise ValueError("motion_consistency_min must be in [0, 1]")
        if self.motion_history_frames < self.motion_confirmation_frames:
            raise ValueError(
                "motion_history_frames must be >= motion_confirmation_frames"
            )


@dataclass(frozen=True)
class ScanSample:
    stamp_ns: int
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]


@dataclass(frozen=True)
class FilteredFrame:
    sample: ScanSample
    ranges: tuple[float, ...]
    dynamic_mask: tuple[bool, ...]
    dynamic_beam_count: int
    active_track_count: int
    dynamic_track_count: int


@dataclass(frozen=True)
class TrackSnapshot:
    track_id: int
    x: float
    y: float
    diameter_m: float
    dynamic: bool
    missed_frames: int


@dataclass(frozen=True)
class _Observation:
    beam_indices: tuple[int, ...]
    centroid_x: float
    centroid_y: float
    diameter_m: float


@dataclass(frozen=True)
class _BufferedSegment:
    track_id: int | None
    beam_indices: tuple[int, ...]


@dataclass(frozen=True)
class _BufferedFrame:
    sample: ScanSample
    segments: tuple[_BufferedSegment, ...]


@dataclass
class _Track:
    track_id: int
    history: deque[tuple[int, float, float]]
    diameter_m: float
    dynamic: bool = False
    missed_frames: int = 0
    hits: int = 1

    @property
    def last_position(self) -> tuple[float, float]:
        _, x, y = self.history[-1]
        return x, y

    def predicted_position(self, stamp_ns: int) -> tuple[float, float]:
        if len(self.history) < 2:
            return self.last_position
        before, latest = self.history[-2], self.history[-1]
        dt_ns = latest[0] - before[0]
        if dt_ns <= 0:
            return self.last_position
        prediction_dt = max(0.0, min((stamp_ns - latest[0]) / 1e9, 0.5))
        vx = (latest[1] - before[1]) / (dt_ns / 1e9)
        vy = (latest[2] - before[2]) / (dt_ns / 1e9)
        return latest[1] + vx * prediction_dt, latest[2] + vy * prediction_dt


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _distance(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


class SessionFilter:
    def __init__(self, config: FilterConfig) -> None:
        self.config = config
        self._tracks: dict[int, _Track] = {}
        self._frames: deque[_BufferedFrame] = deque()
        self._next_track_id = 1
        self._last_stamp_ns: int | None = None
        self._last_pose: Pose2D | None = None
        self.capacity_exceeded = False

    @property
    def track_count(self) -> int:
        return len(self._tracks)

    @property
    def dynamic_track_count(self) -> int:
        return sum(track.dynamic for track in self._tracks.values())

    @property
    def buffered_frame_count(self) -> int:
        return len(self._frames)

    @property
    def track_snapshots(self) -> tuple[TrackSnapshot, ...]:
        return tuple(
            TrackSnapshot(
                track_id=track.track_id,
                x=track.last_position[0],
                y=track.last_position[1],
                diameter_m=track.diameter_m,
                dynamic=track.dynamic,
                missed_frames=track.missed_frames,
            )
            for track in sorted(self._tracks.values(), key=lambda value: value.track_id)
        )

    def reset(self) -> None:
        self._tracks.clear()
        self._frames.clear()
        self._next_track_id = 1
        self._last_stamp_ns = None
        self._last_pose = None
        self.capacity_exceeded = False

    def process_scan(
        self,
        sample: ScanSample,
        pose: Pose2D,
    ) -> FilteredFrame | None:
        self._validate_input(sample, pose)
        observations = self._segment_scan(sample, pose)
        assignments = self._associate(observations, sample.stamp_ns)
        self._frames.append(
            _BufferedFrame(
                sample=sample,
                segments=tuple(
                    _BufferedSegment(track_id, observation.beam_indices)
                    for observation, track_id in zip(observations, assignments)
                ),
            )
        )
        self._last_stamp_ns = sample.stamp_ns
        self._last_pose = pose
        if len(self._frames) <= self.config.buffer_frames:
            return None
        return self._mask_frame(self._frames.popleft())

    def _validate_input(self, sample: ScanSample, pose: Pose2D) -> None:
        if self._last_stamp_ns is not None and sample.stamp_ns <= self._last_stamp_ns:
            raise FilterInputError("scan timestamp is not strictly increasing")
        if sample.stamp_ns < 0 or not sample.ranges:
            raise FilterInputError("scan is empty or has an invalid timestamp")
        geometry = (
            sample.angle_min,
            sample.angle_increment,
            sample.range_min,
            sample.range_max,
        )
        if not all(math.isfinite(value) for value in geometry):
            raise FilterInputError("scan geometry contains non-finite values")
        if sample.angle_increment <= 0.0 or sample.range_min >= sample.range_max:
            raise FilterInputError("scan geometry is invalid")
        valid_beams = sum(
            math.isfinite(value) and sample.range_min <= value <= sample.range_max
            for value in sample.ranges
        )
        if valid_beams < self.config.min_valid_beams:
            raise FilterInputError("scan has too few valid beams")
        if abs(pose.stamp_ns - sample.stamp_ns) > int(
            self.config.max_pose_time_delta_sec * 1e9
        ):
            raise FilterInputError("odometry timestamp is too far from scan timestamp")
        if self._last_pose is None:
            return
        pose_jump = math.hypot(pose.x - self._last_pose.x, pose.y - self._last_pose.y)
        yaw_jump = abs(_wrap_pi(pose.yaw - self._last_pose.yaw))
        if pose_jump > self.config.odom_translation_jump_m:
            raise FilterInputError("odometry translation jump exceeds limit")
        if yaw_jump > self.config.odom_yaw_jump_rad:
            raise FilterInputError("odometry yaw jump exceeds limit")

    def _segment_scan(
        self,
        sample: ScanSample,
        pose: Pose2D,
    ) -> list[_Observation]:
        groups: list[list[tuple[int, float, float]]] = []
        current: list[tuple[int, float, float]] = []
        previous: tuple[float, float] | None = None
        for index, range_m in enumerate(sample.ranges):
            if not (
                math.isfinite(range_m)
                and sample.range_min <= range_m <= sample.range_max
            ):
                if current:
                    groups.append(current)
                    current = []
                previous = None
                continue
            angle = sample.angle_min + index * sample.angle_increment
            point = (range_m * math.cos(angle), range_m * math.sin(angle))
            if previous is not None and _distance(point, previous) > self.config.segment_gap_m:
                groups.append(current)
                current = []
            current.append((index, point[0], point[1]))
            previous = point
        if current:
            groups.append(current)

        observations: list[_Observation] = []
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        for group in groups:
            if len(group) < self.config.min_segment_points:
                continue
            base_points = [(point[1], point[2]) for point in group]
            min_x = min(point[0] for point in base_points)
            max_x = max(point[0] for point in base_points)
            min_y = min(point[1] for point in base_points)
            max_y = max(point[1] for point in base_points)
            diameter = math.hypot(max_x - min_x, max_y - min_y)
            centroid_base_x = sum(point[0] for point in base_points) / len(base_points)
            centroid_base_y = sum(point[1] for point in base_points) / len(base_points)
            centroid_odom_x = (
                pose.x + cos_yaw * centroid_base_x - sin_yaw * centroid_base_y
            )
            centroid_odom_y = (
                pose.y + sin_yaw * centroid_base_x + cos_yaw * centroid_base_y
            )
            observations.append(
                _Observation(
                    beam_indices=tuple(point[0] for point in group),
                    centroid_x=centroid_odom_x,
                    centroid_y=centroid_odom_y,
                    diameter_m=diameter,
                )
            )
        return observations

    def _associate(
        self,
        observations: Sequence[_Observation],
        stamp_ns: int,
    ) -> list[int | None]:
        unmatched_tracks = set(self._tracks)
        assignments: list[int | None] = [None] * len(observations)
        candidates: list[tuple[float, int, int]] = []
        for observation_index, observation in enumerate(observations):
            if not self._is_motion_candidate(observation):
                continue
            position = (observation.centroid_x, observation.centroid_y)
            for track_id, track in self._tracks.items():
                gate = (
                    self.config.dynamic_reacquire_distance_m
                    if track.dynamic and track.missed_frames > 0
                    else self.config.association_distance_m
                )
                distance = _distance(position, track.predicted_position(stamp_ns))
                if distance <= gate:
                    candidates.append((distance, observation_index, track_id))

        used_observations: set[int] = set()
        for _, observation_index, track_id in sorted(candidates):
            if observation_index in used_observations or track_id not in unmatched_tracks:
                continue
            assignments[observation_index] = track_id
            used_observations.add(observation_index)
            unmatched_tracks.remove(track_id)

        for observation_index, track_id in enumerate(assignments):
            observation = observations[observation_index]
            if track_id is None:
                if not self._is_motion_candidate(observation):
                    continue
                track_id = self._new_track(observation, stamp_ns)
                assignments[observation_index] = track_id
            elif track_id in self._tracks:
                self._update_track(self._tracks[track_id], observation, stamp_ns)

        for track_id in unmatched_tracks:
            self._tracks[track_id].missed_frames += 1
        self._expire_static_tracks()
        return assignments

    def _new_track(self, observation: _Observation, stamp_ns: int) -> int | None:
        if len(self._tracks) >= self.config.max_tracks:
            self.capacity_exceeded = True
            return None
        track_id = self._next_track_id
        self._next_track_id += 1
        history = deque(maxlen=self.config.motion_history_frames)
        history.append((stamp_ns, observation.centroid_x, observation.centroid_y))
        self._tracks[track_id] = _Track(
            track_id=track_id,
            history=history,
            diameter_m=observation.diameter_m,
        )
        return track_id

    def _update_track(
        self,
        track: _Track,
        observation: _Observation,
        stamp_ns: int,
    ) -> None:
        track.history.append((stamp_ns, observation.centroid_x, observation.centroid_y))
        track.diameter_m = observation.diameter_m
        track.missed_frames = 0
        track.hits += 1
        if track.dynamic or not self._is_motion_candidate(observation):
            return
        if self._motion_confirmed(track):
            track.dynamic = True

    def _is_motion_candidate(self, observation: _Observation) -> bool:
        return (
            self.config.min_segment_diameter_m
            <= observation.diameter_m
            <= self.config.max_candidate_diameter_m
        )

    def _motion_confirmed(self, track: _Track) -> bool:
        history = list(track.history)
        if len(history) < self.config.motion_confirmation_frames:
            return False
        start = history[0]
        end = history[-1]
        duration_sec = (end[0] - start[0]) / 1e9
        if duration_sec <= 0.0:
            return False
        displacement = math.hypot(end[1] - start[1], end[2] - start[2])
        if displacement < self.config.motion_displacement_m:
            return False
        path_length = sum(
            math.hypot(right[1] - left[1], right[2] - left[2])
            for left, right in zip(history, history[1:])
        )
        steps = [
            (
                (right[0] - left[0]) / 1e9,
                math.hypot(right[1] - left[1], right[2] - left[2]),
            )
            for left, right in zip(history, history[1:])
        ]
        if any(
            gap_sec <= 0.0 or gap_sec > self.config.max_confirmation_gap_sec
            for gap_sec, _ in steps
        ):
            return False
        if any(
            distance / gap_sec > self.config.max_confirmation_step_speed_mps
            for gap_sec, distance in steps
        ):
            return False
        consistency = displacement / path_length if path_length > 0.0 else 0.0
        return (
            displacement / duration_sec >= self.config.min_motion_speed_mps
            and consistency >= self.config.motion_consistency_min
        )

    def _expire_static_tracks(self) -> None:
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if not track.dynamic and track.missed_frames > self.config.max_missed_frames
        ]
        for track_id in expired:
            del self._tracks[track_id]

    def _mask_frame(self, frame: _BufferedFrame) -> FilteredFrame:
        ranges = list(frame.sample.ranges)
        dynamic_mask = [False] * len(ranges)
        for segment in frame.segments:
            track = self._tracks.get(segment.track_id) if segment.track_id is not None else None
            if track is None or not track.dynamic:
                continue
            for beam_index in segment.beam_indices:
                ranges[beam_index] = float("nan")
                dynamic_mask[beam_index] = True
        dynamic_beam_count = sum(dynamic_mask)
        return FilteredFrame(
            sample=frame.sample,
            ranges=tuple(ranges),
            dynamic_mask=tuple(dynamic_mask),
            dynamic_beam_count=dynamic_beam_count,
            active_track_count=len(self._tracks),
            dynamic_track_count=self.dynamic_track_count,
        )
