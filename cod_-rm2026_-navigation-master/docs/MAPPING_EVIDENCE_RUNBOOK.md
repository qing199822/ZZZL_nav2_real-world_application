# Mapping Evidence Runbook

This procedure captures the Stage 0 C1 evidence used to decide whether the
MID-360 two-dimensional projection and same-source Point-LIO odometry are
adequate for later dynamic-obstacle filtering.

Stage 0 does not change the production `/scan -> slam_toolbox` path, Nav2,
map saving, or collision behavior.

## Build

```bash
cd /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_dynamic_filter --symlink-install
source install/setup.bash
```

## Start The Robot Stack

Start the existing real-world stack as usual so these required streams are
available: `/livox/lidar`, `/livox/lidar_filtered`, `/livox/imu`,
`/Odometry`, `/tf`, and `/tf_static`.

For the current repository this means starting the MID-360 driver first and
then the unchanged multi-point navigation launch:

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
source /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch cod_bringup multiplenav_launch.py
```

Start the separate evidence projector. It consumes
`/livox/lidar_filtered` and publishes only `/scan_raw`:

```bash
# Terminal 3
source /opt/ros/humble/setup.bash
source /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch slam_dynamic_filter evidence_scan.launch.py
```

## Preflight

```bash
# Terminal 4
source /opt/ros/humble/setup.bash
source /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
sdfctl doctor
ros2 topic info /scan_raw
ros2 topic hz /scan_raw
```

Expected `/scan_raw` type: `sensor_msgs/msg/LaserScan`. Expected approximate
rate: 8-12 Hz. Do not start recording unless doctor prints:

```text
PASS preflight: required topics and disk available
```

## Record The 120-Second Scenario

Keep people, other robots, carts, and other moving objects outside the 8 m
measurement zone for the entire run.

- 0-20 s: remain stationary facing a textured static wall.
- 20-40 s: yaw slowly left then right, at no more than 0.4 rad/s, keeping the
  same wall visible.
- 40-60 s: move slowly forward then backward, at no more than 0.3 m/s, and
  remain at least 1 m from the wall.
- 60-80 s: move laterally left then right at no more than 0.3 m/s.
- 80-100 s: combine low-speed translation and yaw within the limits above.
- 100-120 s: remain stationary near the starting pose.

Run:

```bash
sdfctl record --duration-sec 120
```

Follow the printed `SCENARIO` transitions. Preserve the generated value from
the line:

```text
SESSION <session_id>
```

Do not use the old 7.84-second CustomMsg bag. It lacks the required topic and
type contract and is intentionally rejected.

## Analyze

```bash
sdfctl analyze --session <session_id>
```

The expected outcomes are:

| Exit | Meaning |
| ---: | --- |
| 0 | `PASS C1`: complete evidence passes every locked check. |
| 5 | Invalid or incomplete evidence; fix the capture and repeat it. |
| 6 | `BLOCKED C1`: evidence is valid but one or more thresholds failed. |
| 7 | Unexpected internal error; inspect the terminal output. |

A valid C1 block is a result, not permission to tune thresholds around the
robot. Any threshold change requires a separate reviewed plan.

## Verify Artifacts

```bash
find "/home/wangtao/cod_mapping_sessions/<session_id>" -maxdepth 2 -type f | sort
python3 -m json.tool "/home/wangtao/cod_mapping_sessions/<session_id>/reports/stage0_metrics.json" >/dev/null
python3 -m json.tool "/home/wangtao/cod_mapping_sessions/<session_id>/evidence/c1_gate.json" >/dev/null
```

Expected files include `metadata.json`, `record_manifest.json`,
`system_profile.jsonl`, the frozen `config/stage0_thresholds.yaml`, rosbag
metadata/data, `reports/stage0_metrics.json`, and `evidence/c1_gate.json`.

If a run is aborted, verify the exact generated session ID and move only that
incomplete session to the desktop trash:

```bash
gio trash "/home/wangtao/cod_mapping_sessions/<aborted_session_id>"
```

Do not delete successful evidence sessions. C1 remains pending until this
full on-robot procedure has been completed and its report reviewed.
