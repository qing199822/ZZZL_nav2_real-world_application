from __future__ import annotations

import copy
import math
import time
import uuid
from collections import deque

import message_filters
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .bag_reader import Pose2D
from .session_filter import (
    FilterConfig,
    FilteredFrame,
    FilterInputError,
    ScanSample,
    SessionFilter,
)


CORE_PARAMETER_DEFAULTS = {
    "buffer_frames": 10,
    "segment_gap_m": 0.30,
    "min_segment_points": 3,
    "min_segment_diameter_m": 0.18,
    "max_candidate_diameter_m": 1.50,
    "association_distance_m": 0.75,
    "dynamic_reacquire_distance_m": 0.85,
    "motion_displacement_m": 0.30,
    "min_motion_speed_mps": 0.12,
    "max_confirmation_step_speed_mps": 1.20,
    "max_confirmation_gap_sec": 0.15,
    "motion_confirmation_frames": 10,
    "motion_consistency_min": 0.90,
    "motion_history_frames": 12,
    "max_missed_frames": 4,
    "max_tracks": 32,
    "min_valid_beams": 3,
    "max_pose_time_delta_sec": 0.15,
    "odom_translation_jump_m": 0.50,
    "odom_yaw_jump_rad": 0.50,
}


def build_output_scans(
    source: LaserScan,
    frame: FilteredFrame,
    mode: str,
) -> tuple[LaserScan, LaserScan]:
    if mode not in ("observe", "enforce"):
        raise ValueError(f"unsupported filter mode: {mode}")
    if len(source.ranges) != len(frame.ranges):
        raise ValueError("source scan and filtered frame have different beam counts")

    output = copy.deepcopy(source)
    if mode == "enforce":
        output.ranges = list(frame.ranges)

    dynamic_mask = copy.deepcopy(source)
    dynamic_mask.ranges = [
        float(source.ranges[index]) if dynamic else float("nan")
        for index, dynamic in enumerate(frame.dynamic_mask)
    ]
    dynamic_mask.intensities = []
    return output, dynamic_mask


def filter_output_allowed(*, frozen: bool, capacity_exceeded: bool) -> bool:
    return not frozen and not capacity_exceeded


def _yaw_from_odometry(message: Odometry) -> float:
    quaternion = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _stamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SlamDynamicFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("slam_dynamic_filter_node")
        for name, default in CORE_PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter("mode", "observe")
        self.declare_parameter("sync_slop_sec", 0.12)
        self.declare_parameter("scan_stale_timeout_sec", 1.0)
        self.declare_parameter("diagnostic_period_sec", 0.5)

        core_values = {
            name: self.get_parameter(name).value
            for name in CORE_PARAMETER_DEFAULTS
        }
        self._engine = SessionFilter(FilterConfig(**core_values))
        self._mode = str(self.get_parameter("mode").value)
        if self._mode not in ("observe", "enforce"):
            raise ValueError("mode must be 'observe' or 'enforce'")

        self._session_id = uuid.uuid4().hex
        self._source_scans: deque[LaserScan] = deque()
        self._frozen = False
        self._ready = False
        self._latest_linear_speed: float | None = None
        self._latest_angular_speed: float | None = None
        self._last_scan_monotonic: float | None = None
        self._input_errors = 0
        self._published_scans = 0

        self._filtered_publisher = self.create_publisher(
            LaserScan, "scan_slam_filtered", qos_profile_sensor_data
        )
        self._mask_publisher = self.create_publisher(
            LaserScan, "dynamic_mask", qos_profile_sensor_data
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", 10
        )
        ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ready_publisher = self.create_publisher(Bool, "ready", ready_qos)
        self._track_publisher = self.create_publisher(MarkerArray, "tracks", 2)

        scan_subscriber = message_filters.Subscriber(
            self,
            LaserScan,
            "scan_raw",
            qos_profile=qos_profile_sensor_data,
        )
        odom_subscriber = message_filters.Subscriber(
            self,
            Odometry,
            "odometry",
            qos_profile=qos_profile_sensor_data,
        )
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [scan_subscriber, odom_subscriber],
            queue_size=20,
            slop=float(self.get_parameter("sync_slop_sec").value),
        )
        self._synchronizer.registerCallback(self._on_synced_scan)

        self.create_service(Trigger, "reset_session", self._on_reset)
        self.create_service(SetBool, "freeze", self._on_freeze)
        self.create_timer(
            float(self.get_parameter("diagnostic_period_sec").value),
            self._publish_diagnostics,
        )
        self._publish_ready(False)
        self.get_logger().info(
            f"session {self._session_id} started in {self._mode} mode"
        )

    def _on_synced_scan(self, scan: LaserScan, odometry: Odometry) -> None:
        scan_stamp_ns = _stamp_ns(scan)
        odom_stamp_ns = _stamp_ns(odometry)
        sample = ScanSample(
            stamp_ns=scan_stamp_ns,
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            ranges=tuple(float(value) for value in scan.ranges),
        )
        twist = odometry.twist.twist
        pose = Pose2D(
            stamp_ns=odom_stamp_ns,
            x=float(odometry.pose.pose.position.x),
            y=float(odometry.pose.pose.position.y),
            yaw=_yaw_from_odometry(odometry),
            vx=float(twist.linear.x),
            vy=float(twist.linear.y),
            wz=float(twist.angular.z),
        )
        self._latest_linear_speed = math.hypot(pose.vx or 0.0, pose.vy or 0.0)
        self._latest_angular_speed = abs(pose.wz or 0.0)

        try:
            result = self._engine.process_scan(sample, pose)
        except FilterInputError as error:
            self._input_errors += 1
            self._set_ready(False)
            self.get_logger().warning(f"scan rejected: {error}")
            return

        self._last_scan_monotonic = time.monotonic()
        self._source_scans.append(copy.deepcopy(scan))
        if result is None:
            return
        source = self._source_scans.popleft()
        if not filter_output_allowed(
            frozen=self._frozen,
            capacity_exceeded=self._engine.capacity_exceeded,
        ):
            self._set_ready(False)
            return
        output, mask = build_output_scans(source, result, self._mode)
        self._filtered_publisher.publish(output)
        self._mask_publisher.publish(mask)
        self._published_scans += 1
        self._set_ready(not self._engine.capacity_exceeded)
        self._publish_tracks()

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response):
        stopped = (
            self._latest_linear_speed is not None
            and self._latest_angular_speed is not None
            and self._latest_linear_speed <= 0.03
            and self._latest_angular_speed <= 0.03
        )
        if not stopped:
            response.success = False
            response.message = "reset denied: robot must be stopped"
            return response
        self._engine.reset()
        self._source_scans.clear()
        self._session_id = uuid.uuid4().hex
        self._input_errors = 0
        self._published_scans = 0
        self._set_ready(False)
        response.success = True
        response.message = f"new session: {self._session_id}"
        return response

    def _on_freeze(self, request: SetBool.Request, response: SetBool.Response):
        self._frozen = bool(request.data)
        self._set_ready(False if self._frozen else self._ready)
        response.success = True
        response.message = "filter frozen" if self._frozen else "filter resumed"
        return response

    def _set_ready(self, value: bool) -> None:
        if self._ready == value:
            return
        self._ready = value
        self._publish_ready(value)

    def _publish_ready(self, value: bool) -> None:
        message = Bool()
        message.data = value
        self._ready_publisher.publish(message)

    def _publish_tracks(self) -> None:
        messages = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        messages.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for track in self._engine.track_snapshots:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = stamp
            marker.ns = "slam_dynamic_filter_tracks"
            marker.id = track.track_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = track.x
            marker.pose.position.y = track.y
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(0.10, track.diameter_m)
            marker.scale.y = max(0.10, track.diameter_m)
            marker.scale.z = 0.10
            marker.color.a = 0.85
            marker.color.r = 1.0 if track.dynamic else 1.0
            marker.color.g = 0.0 if track.dynamic else 0.75
            messages.markers.append(marker)
        self._track_publisher.publish(messages)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        stale_timeout = float(self.get_parameter("scan_stale_timeout_sec").value)
        stale = (
            self._last_scan_monotonic is None
            or now - self._last_scan_monotonic > stale_timeout
        )
        level = DiagnosticStatus.OK
        message = "healthy"
        if self._engine.capacity_exceeded:
            level = DiagnosticStatus.ERROR
            message = "track capacity exceeded"
        elif self._frozen:
            level = DiagnosticStatus.WARN
            message = "frozen"
        elif stale:
            level = DiagnosticStatus.WARN
            message = "scan/odometry input stale"
        elif not self._ready:
            level = DiagnosticStatus.WARN
            message = "warming"

        status = DiagnosticStatus()
        status.level = level
        status.name = "slam_dynamic_filter/session"
        status.hardware_id = "mid360-pointlio"
        status.message = message
        values = {
            "session_id": self._session_id,
            "mode": self._mode,
            "ready": str(self._ready).lower(),
            "frozen": str(self._frozen).lower(),
            "active_tracks": str(self._engine.track_count),
            "dynamic_tracks": str(self._engine.dynamic_track_count),
            "buffered_frames": str(self._engine.buffered_frame_count),
            "published_scans": str(self._published_scans),
            "input_errors": str(self._input_errors),
            "capacity_exceeded": str(self._engine.capacity_exceeded).lower(),
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostic_publisher.publish(diagnostics)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamDynamicFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
