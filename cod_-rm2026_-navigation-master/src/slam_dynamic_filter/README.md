# slam_dynamic_filter

Stage 0 evidence tooling plus an opt-in pragmatic session-memory filter for
the MID-360 scan and same-source Point-LIO odometry.

It records and analyzes `/scan_raw`, `/Odometry`, `/tf`, `/tf_static`, Livox
point clouds, Livox IMU data, and a one-hertz system profile. Results are
written below `~/cod_mapping_sessions/<session_id>/`.

## Commands

```bash
ros2 launch slam_dynamic_filter evidence_scan.launch.py
sdfctl doctor
sdfctl record --duration-sec 120
sdfctl analyze --session <session_id>
```

`record` prints the generated session ID. `analyze` returns `0` when C1
passes, `6` when valid evidence fails one or more thresholds, and `5` when
the evidence itself is invalid.

## Scope

Stage 0 is evidence only. It does not publish a tracker, mask or remap
`/scan`, change `slam_toolbox` or Nav2, save maps, replace autosave, or enable
an enforce mode. A complete 120-second on-robot run is required before C1 can
be considered evaluated.

## Pragmatic mapping filter

The filter can run standalone, or through the production mapping launch's
explicit opt-in mode:

```bash
ros2 launch slam_dynamic_filter mapping_filter.launch.py mode:=observe
ros2 launch slam_dynamic_filter mapping_filter.launch.py mode:=enforce
ros2 launch cod_bringup multiplenav_launch.py slam_filter_mode:=observe
ros2 launch cod_bringup multiplenav_launch.py slam_filter_mode:=enforce
```

`observe` publishes the delayed scan unchanged and exposes the computed mask.
`enforce` publishes NaN-masked dynamic returns on `/scan_slam_filtered`.
Neither mode changes Nav2 costmap inputs. An object must be observed moving
before it can be remembered as dynamic; occupancy fused before that motion is
not retroactively removed from a running slam_toolbox map.

The production default is `slam_filter_mode:=disabled`. Its existing automatic
map saver is disabled in observe/enforce modes, so filtered candidates require
an explicit operator save after map inspection. See
`docs/PRAGMATIC_MAPPING_FILTER.md` in the workspace.
