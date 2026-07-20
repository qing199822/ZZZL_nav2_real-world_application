<!-- /autoplan restore point: /home/wangtao/.gstack/projects/qing199822-ZZZL_nav2_real-world_application/improve-autoplan-restore-20260719-201333.md -->
# SLAM Dynamic Obstacle Filter Autoplan Review

Reviewed source: docs/superpowers/specs/2026-07-19-slam-dynamic-obstacle-filter-design.md
Branch: improve
Commit: dfe6fa6
Base: origin/master (warning: no merge-base with HEAD; review is file/context based)
Target: ROS 2 Humble on AMD Ryzen 5 4500U, no discrete GPU

## Approved Design Summary

The approved design adds an online, CPU-only 2D `LaserScan` temporal filter before `slam_toolbox`, using odom-frame motion compensation so dynamic/static decisions are not polluted by SLAM map feedback. The filter buffers roughly 0.8-1.2 seconds of scans, confirms motion across multiple frames, and retroactively masks buffered returns once a target is confirmed dynamic.

Dynamic identity is retained for the full mapping session: a target that has moved remains dynamic even if it later stops, with support for occlusion, nearby reactivation, and beam-history tracking. This is session memory only, not a permanent forbidden-space map.

Masking is conservative. Confirmed dynamic returns and unobserved space behind them are emitted as `NaN`; the system never inserts fake `inf` free-space readings behind an obstacle. Static or unconfirmed returns pass through unchanged, real walls may reappear when observed again, and real no-return readings remain `inf`.

Nav2 uses a direct, no-wait branch from the filtered point cloud into STVL for runtime obstacle handling. The delayed temporal scan stream is only for SLAM, so navigation avoidance is not slowed by the 0.8-1.2 second confirmation window. STVL remains useful for costmap decay but is not treated as a substitute for SLAM input filtering.

Map saving is gated by `safe_map_saver`, fresh diagnostics, matching session ID, healthy filter state, adequate filtered data, bag and disk health, low robot motion, no backlog, low TF failure rate, and an explicit save or explicitly enabled healthy snapshot behavior. Missing, stale, mismatched, or non-OK diagnostics block saving. Unconditional final auto-save is removed; snapshots are candidates only, and final maps require offline refinement and review.

Mapping mode records the raw scan, dynamic mask, diagnostics, TF, odometry, frozen parameters, package and git revision, and session metadata. Offline final refinement deterministically recomputes tracks forward and backward from `/scan_raw`, TF, and odometry using the frozen parameters, masks early scans that were only later identified as dynamic, reruns SLAM with the same parameters, and emits candidate map files, quality reports, and reproducibility metadata without overwriting original data.

Delivery is staged. Stage A delivers the online safety path, package wiring, diagnostics, gates, saver, unit tests, Gazebo tests, and R5 performance validation. Stage B collects and calibrates R5-4500U bags for static, single-person, multi-person, and robot scenarios with frozen configuration. Stage C delivers offline forward/backward refinement, offline SLAM, quality reporting, and manual final promotion. Stage A must remain safe even before Stage C exists.

The design explicitly accepts the physical limitation that lidar-only geometry cannot identify a person, robot, box, or fixed object that never moves during the session. It does not promise semantic classification, permanent identities, or cross-session dynamic memory.

R5-4500U criteria require bounded CPU and memory use: average filter load no more than 50% of one core, short peaks only, less than 200 MB extra RSS, p95 frame processing under 30 ms, p95 SLAM delay within 0.8-1.3 seconds, Nav2 avoidance added latency under 10 ms, bounded queues, no leaks or crashes over two hours, and offline work capped to two cores. Map-quality criteria require major reductions in persistent dynamic false occupancy while preserving walls and limiting unknown-area growth, with deterministic evidence for any unmet real-world metric.

## Decision Audit Trail

| # | Phase | Decision | Classification | Principle | Rationale | Rejected |
|---|---|---|---|---|---|---|
| 1 | Phase 1 | 采用 SELECTIVE_EXPANSION 审查模式 | Accepted | 只扩大直接爆炸半径 | CEO 审查需要补齐安全、证据、回滚、性能和可复现性，但不扩大到语义感知或云端处理。 | 全量重设需求；只做窄实现清单 |
| 2 | Phase 1 | 保留完整 online + offline 目标架构 | Accepted | 尊重已批准方向 | 用户已确认高层设计，在线过滤、会话记忆、安全保存、离线重放仍是目标；可行性证据作为最终门禁处理。 | 静默降级为纯离线或纯定位 |
| 3 | Phase 1 | 复用现有组件并最小改线 | Accepted | 仓库优先 | `cpp_lidar_filter`、`pointcloud_to_laserscan`、`small_point_lio`、STVL、`slam_toolbox`、`nav2_map_server`、Gazebo 和 bag 脚本已有明确接入点。 | 重写传感器栈；替换 SLAM |
| 4 | Phase 1 | 地图保存不使用 shell 子进程 | Accepted | 类型化服务优先 | 当前 autosave 通过 `bash -c` 调 `map_saver_cli`，有路径、状态和注入风险；应改为 `nav2_map_server` 类型化服务客户端。 | 继续 `ExecuteProcess(['bash','-c',...])` |
| 5 | Phase 1 | 固定输出根目录并使用生成的 session_id | Accepted | 可追溯与路径约束 | 候选地图、bag、metadata、report 必须由同一 session 关联，并限制在固定 root 的 canonical path 内。 | 用户传任意保存路径；跨 session 混存 |
| 6 | Phase 1 | 所有在线缓冲队列有上界和溢出策略 | Accepted | 失败可见且有界 | R5-4500U 目标要求 bounded queues；溢出进入 DEGRADED/FROZEN 并阻断保存，而不是隐性延迟扩大。 | 无界 scan/track/session 缓冲 |
| 7 | Phase 1 | rollout 暴露 `disabled|observe|enforce` 三种模式 | Accepted | 渐进启用 | `disabled` 用现状基线但禁止最终地图，`observe` 只出诊断/掩码且阻断最终保存，`enforce` 才路由过滤 scan。 | 一次性切换到 enforce |
| 8 | Phase 1 | 增加整机性能预算 | Accepted | 控制链优先 | 单进程 filter 预算不足以覆盖 Nav2、SLAM、TF、bag、温控和控制频率抖动；需同时量测总 CPU、RSS、cmd_vel latency、TF latency 和 2h throttling。 | 只看 filter p95/CPU |
| 9 | Phase 1 | Gazebo 仅作为二级验收 | Accepted | 传感器真实性优先 | 仓库文档和模型说明 Gazebo 是 360 单层 10Hz 近似，不代表 MID-360 非重复 pattern；真实 bag 是可行性门禁证据。 | 用 Gazebo 通过替代真机证据 |
| 10 | Phase 1 | 不引入无关语义扩展 | Accepted | 目标边界 | 不加入 camera semantic、网络分类、云处理、跨 session 动态类别记忆；本阶段只解决动态污染、安全保存和可复现证据。 | 人/车/箱分类；远端训练管线 |
| 11 | Phase 3 | 扩展 evidence recorder 记录 raw/filtered cloud、MID-360 built-in IMU、scan、mask、diagnostics、TF、odom 和冻结元数据 | Accepted | 证据先于复杂度 | Phase 3 工程审查确认现有 bag 默认只录 `/livox/lidar /livox/imu`，不足以证明投影、过滤、SLAM 输入和 replay 可复现性；recorder 必须同时保存实际 MID-360 built-in IMU topic 和 driver config。 | 只录 raw cloud 后离线猜测其余输入 |
| 12 | Phase 3 | 使用 typed save/reset/mode services | Accepted | 类型化边界 | `SaveCandidateMap`、`ResetFilterSession`、`SetFilterMode` 绑定 request_id/session_id 和枚举状态，替代自由文本命令。 | 参数开关加 shell 脚本 |
| 13 | Phase 3 | `DiagnosticArray` 仅观测，不作为 saver 权威状态 | Accepted | 状态单一来源 | `safe_map_saver` 读取共享 `SessionHealth` 和 typed request；诊断文本只给 operator/RViz/日志使用，避免解析自由文本。 | saver 解析 diagnostics message |
| 14 | Phase 3 | 中央 metadata schema version `1` | Accepted | 可复现契约 | online/offline/report/map candidate 使用同一 schema 版本和冻结配置；schema mismatch 阻断 final promotion。 | 每个工具私有 metadata 字段 |
| 15 | Phase 3 | 确定 deterministic replay stable order/time contract | Accepted | 时间语义显式 | replay 以 `(source timestamp, topic priority, bag sequence)` 排序，preserve duplicate sequence，`use_sim_time` 和固定 `/clock`，禁止 wall-clock 决策。 | rosbag 默认顺序即视为确定 |
| 16 | Phase 3 | NaN/inf SLAM 行为必须做集成测试 | Accepted | 不假设下游语义 | masking 依赖 `NaN` 表示 unknown/dynamic、`inf` 表示真实 no-return；`slam_toolbox` 实际处理必须通过 fixture 验证。 | 单元测试 mask 后直接认为 SLAM 正确 |
| 17 | Phase 3 | Gazebo smoke-only，不作传感器可行性验收 | Accepted | 真实 bag 为准 | Gazebo 可覆盖 launch、QoS、状态机和粗行为，但不能替代 MID-360 非重复投影证据。 | Gazebo 通过即进入 enforce |
| 18 | Phase 3 | 增加整机 provisional gates | Accepted | 控制链优先 | R5 验收同时看 total CPU、single-core pin、controller loop、cmd_vel added latency、scan drop、TF latency、thermal、RSS 和 bag jitter。 | 只保留 filter 单进程预算 |
| 19 | Phase 3 | 禁止 shell saver | Accepted | 最小攻击面 | 当前 `auto_save_map` 的 `bash -c` 路径必须删除或 disabled wrapper，不复制到新实现。 | 包装 `map_saver_cli` 继续使用 shell |
| 20 | Phase 3 | workstreams staged，不把 >8 files/>2 components 做成一个 PR | Accepted | 降低集成风险 | 范围触及新包、bringup、Gazebo、scripts、docs 和测试，只能按 foundation/lanes/merge/enforce/offline 顺序交付。 | 单 PR 同时改所有组件 |
| 21 | Phase 3 | 硬件边界：仅 MID-360 built-in IMU 可用；电控 IMU 未连接；无独立轮速里程计 | User-confirmed constraint | factual hardware constraint | 用户确认当前硬件只有 MID-360 内置 IMU，`small_point_lio` `/Odometry` 来自 MID-360 point cloud + built-in IMU，属于 same-source odom；C1 不能要求 wheel/electrical IMU 独立对比。 | 把独立 wheel/electrical IMU 当作当前验收必需项 |
| 22 | Phase 3.5 | 增加 `sdfctl` wrapper 作为 operator 唯一入口 | Accepted | 降低 ROS 手工操作摩擦 | CLI 封装 typed ROS services、topic preflight、health 和 evidence commands；operator 不手写 service YAML。 | 继续要求手动 `ros2 service call` YAML |
| 23 | Phase 3.5 | 固定 DX hard defaults | Accepted | fail-closed 默认 | `mode=observe`、`final_promotion=false`、`candidate_snapshot=false`、`output_root=$HOME/cod_mapping_sessions`、`max_record_duration_sec=300`、`min_disk_free_gb=20`、`require_c1_for_enforce=true`，正常比赛导航默认 reviewed static maps。 | 安装后默认 enforce 或自动 candidate |
| 24 | Phase 3.5 | C1 gate file 成为 enforce 唯一证据入口 | Accepted | 机器可审计证据 | `~/cod_mapping_sessions/<id>/evidence/c1_gate.json` 记录 stability、same-source odom、R5 profile、schema/config/git 和 `allow_enforce`；不匹配则 enforce impossible。 | 解析日志文本决定 enforce |
| 25 | Phase 3.5 | 固定 session artifact layout | Accepted | 可复现和可回滚 | 所有 evidence、bag、config、logs、reports、candidate maps、refined outputs、active map manifest 都写入 `~/cod_mapping_sessions/<session_id>/`，禁止直接写 `cod_bringup/maps`。 | 直接覆盖 bringup maps |
| 26 | Phase 3.5 | 建立单一 Mapping DX runbook | Accepted | 减少文档路径分裂 | `docs/MAPPING_DX_RUNBOOK.md` 从 README、QUICK_START、Livox debug docs 链接，按 build->doctor->record->analyze->observe->diagnose->enforce decision->save->refine->promote->rollback->upgrade 排列。 | 分散保留 raw map_saver 教程 |
| 27 | Phase 3.5 | 规划 metadata schema migration contract | Accepted | 升级可维护 | v1 当前无需迁移；v2 出现后 backward reader 支持前一版、dry-run migration、迁移测试和 release checklist。 | schema 变更后旧 session 不可 refine |
| 28 | Phase 3.5 | 增加 synthetic CI fixtures 和 mini bag | Accepted | DX 可度量 | CI 覆盖 build/unit/launch/fake saver/schema/replay，真实 bag 因隐私和体积外置。 | 只靠真实实验室 bag 手工验证 |
| 29 | Phase 3.5 | 所有 operator 错误必须 actionable | Accepted | 缩短诊断时间 | 输出包含 code、problem、measured cause vs threshold、exact fix command/file、retry command、docs anchor；diagnostics keys 不是决策源。 | 输出自由文本或只给 ROS diagnostics |

## Phase 1 — CEO Review

### 0A 前提挑战

用户确认了已批准的高层前提：在线过滤、会话记忆、安全保存、离线重放和 R5 CPU-only 约束仍保留。以下三项可行性发现不静默改变已批准设计，而形成最终门禁的 User Challenge：投影 Livox 是否能作为稳定 2D scan、same-source LIO odom 是否足够可用、仅按 R5 filter 预算是否足够。

| Premise | Repo evidence | Verdict | CEO challenge |
|---|---|---|---|
| dynamic occupancy pollution is real and verified by `multiplenav` launch -> `pointcloud_to_laserscan` -> `slam_toolbox` plus static-layer max-combine | 真机 launch 中 `pointcloud_to_laserscan` 从 `/livox/lidar` 输出 `/scan`，随后 `slam_toolbox` 启动：`cod_-rm2026_-navigation-master/src/cod_bringup/launch/multiplenav_launch.py:65-94`；Nav2 local/global costmap 同时加载 `static_layer` 与 STVL，且 STVL `combination_method: 1` 为 max combine：`cod_-rm2026_-navigation-master/src/cod_bringup/params/multiplenav2_params.yaml:186-205`、`:275-295`。 | Valid | 动态物体污染 SLAM 地图的风险链路成立，过滤 SLAM 输入的目标合理。 |
| projected Livox acts as repeatable ~720-beam 10Hz planar lidar | `angle_increment: 0.0087` 在 360 度下约 722 beams：`multiplenav_launch.py:74-77`；但 converter 强制 `time_increment = 0.0`：`pointcloud_to_laserscan_node.cpp:147-151`；真实安装有 45 度 pitch：`multiplenav_launch.py:123-131`；Gazebo 文档说明仅 360 pts/rev、10Hz、单环近似，非 MID-360 非重复 pattern：`docs/livox-debug-guide.md:72-80`。 | UNPROVEN feasibility premise | 必须用真实 bag 验证 beam 稳定性、角向覆盖、帧间重复度、scan rate 和动态遮挡形态。 |
| same-source LIO odom is usable enough for ego-motion compensation | `small_point_lio` 订阅同一 Livox/MID-360 built-in IMU 后 `handle_once()`，再发布 `/Odometry` 与 `/cloud_registered`：`small_point_lio_node.cpp:197-210`、`:26-28`；odometry callback 发布 odom->base_link：`:52-99`。 | UNPROVEN / same-source | `/Odometry` 来自 MID-360 point cloud + built-in IMU，属于 same-source，不是独立 wheel/electrical IMU 证据。C1 证据改为真实 bag 的 static-wall residual、动态目标前后 LIO continuity/drift、直行/横移/旋转中的 compensated static consistency；若证据失败，保持 observe，绝不 enforce。 |
| Nav2 safety remains intact when SLAM pauses | STVL 仍直接消费 `/livox/lidar_filtered`：`multiplenav2_params.yaml:212-229`、`:302-320`；但未发现 `collision_monitor` 配置，STVL `expected_update_rate: 0.0` 不做 freshness 移除：`:225`、`:315`；速度上限达到 2.5 m/s：`:93-99`、`:465-475`。 | Partially valid | SLAM 暂停不应拖慢 STVL 分支，但还缺少 sensor freshness stop / collision monitor / lifecycle gating。 |
| deterministic offline replay is valid | 现有 `record_bag.sh` 有 `set -euo pipefail`、`timeout` 和 metadata 检查：`scripts/record_bag.sh:23`、`:181-200`，但默认只录 `/livox/lidar /livox/imu`：`:25-29`。 | Valid only with constraints | 必须冻结 params、code/git revision、完整 bag、稳定排序/time 语义，并新增重放结果一致性测试。 |
| R5 filter budget alone is sufficient | 计划中已有 filter CPU/RSS/p95 目标；实际系统还同时运行 SLAM、Nav2 MPPI 50Hz、STVL 20Hz、velocity smoother 30Hz 和 bag 写盘：`multiplenav2_params.yaml:60-87`、`:172-176`、`:465-475`。 | Incomplete | 需要整机 thermal/control latency 预算，不能只按 filter 进程验收。 |

### 0B What already exists

| Existing asset | Repo path/lines | Reuse / replace decision | Notes |
|---|---|---|---|
| `cpp_lidar_filter` crop / optional voxel | 参数声明、CropBox 和 optional VoxelGrid 在 `cod_-rm2026_-navigation-master/src/cpp_lidar_filter/src/filter_node.cpp:15-27`、`:102-136`。 | Reuse | 继续作为车身裁剪和轻量降采样节点；动态过滤不塞入该节点，避免职责混淆。 |
| `pointcloud_to_laserscan` | 真机 remap 当前为 `/livox/lidar` -> `/scan`：`multiplenav_launch.py:65-83`；核心投影代码在 `pointcloud_to_laserscan_node.cpp:137-230`。 | Reuse but measure and reroute | 量测输出 scan 后，将 SLAM 输入改为 filtered cloud 路径生成的 scan；保留现有包。 |
| `small_point_lio` `/Odometry` and `/cloud_registered` | publishers 在 `small_point_lio_node.cpp:26-28`，odom TF 发布在 `:52-99`，registered cloud 发布在 `:101-170`。 | Reuse as same-source candidate odom | `/Odometry` 使用 MID-360 point cloud + built-in IMU；可作为 ego-motion candidate，但必须验证与 Livox 点云动态污染的耦合。 |
| Nav2 STVL marking / clearing | local/global STVL source 使用 `/livox/lidar_filtered`，`marking: true`、`clearing: true`：`multiplenav2_params.yaml:212-229`、`:302-320`。 | Reuse for no-delay avoidance | 保持不经 0.8-1.2s 延迟的安全分支；补 freshness/stop gate。 |
| `slam_toolbox` and `nav2_map_server` | `slam_toolbox` launch 在 `multiplenav_launch.py:85-94`；`map_saver` 参数在 `multiplenav2_params.yaml:356-362`。 | Reuse services | `safe_map_saver` 用类型化 service，不再 shell 调 `map_saver_cli`。 |
| `auto_save_map` launch | Timer + `ExecuteProcess` + `bash -c` 在 `auto_save_map.launch.py:15-27`；真机 launch 无条件 include：`multiplenav_launch.py:181-185`。 | Replace unconditional save semantics | 改成候选保存、健康门禁和显式 final promotion。 |
| `scripts/record_bag.sh` | strict shell、driver wait、timeout、bag metadata 检查在 `scripts/record_bag.sh:23`、`:141-160`、`:181-200`。 | Reuse patterns, create mapping-specific recorder | 新 recorder 录 raw scan、filtered scan/mask、diag、TF、odom、params、git metadata。 |
| Gazebo launch/world | Gazebo SLAM path和 autosave arg 在 `gazebo_slam.launch.py:94-129`、`:142-148`；仿真模型说明单环近似在 `docs/livox-debug-guide.md:72-80`。 | Wiring / gross behavior only | 可验 launch、QoS、状态机、基础 masking，不用于 sensor-fidelity acceptance。 |

### 0C Dream state

```text
CURRENT
  /livox/lidar
    ├─ cpp_lidar_filter -> /livox/lidar_filtered -> Nav2 STVL
    ├─ pointcloud_to_laserscan(/livox/lidar) -> /scan -> slam_toolbox -> /map
    └─ small_point_lio -> /Odometry
  auto_save_map: timer -> bash -c -> map_saver_cli

THIS DESIGN
  /livox/lidar -> cpp_lidar_filter -> /livox/lidar_filtered
    ├─ immediate Nav2 STVL marking/clearing
    └─ measured pointcloud_to_laserscan -> /scan_raw
         + candidate /Odometry + TF
         -> dynamic_obstacle_filter observe/enforce
         -> /scan_slam_filtered -> slam_toolbox
  SessionHealth + diagnostics -> safe_map_saver -> candidate maps only
  mapping recorder -> frozen bag + params + git metadata -> deterministic offline refinement

12-MONTH IDEAL
  reviewed static maps are used for competition navigation
  runtime dynamic obstacles live only in costmaps, not permanent maps
  scan/odom feasibility evidence is measured and versioned
  bags/maps/reports reproduce byte-stable or tolerance-stable results
  lifecycle and collision gates stop unsafe operation before stale sensors reach cmd_vel
  online tracker is enabled only where real-bag validation proves it earns its complexity
```

### 0C-bis Alternatives

| Approach | Description | Effort | Risk | Completeness |
|---|---|---:|---:|---:|
| A minimal map-safety/evidence first | Route SLAM through filtered-cloud path, remove unconditional autosave, allow candidate-only manual gated save, add deterministic mapping bag capture, run real-bag scan/odom metrics before tracker. | M | Low/Med | 6/10 |
| B approved online+offline architecture | Odom-compensated 2D tracker, session memory, safe saver, offline replay/refinement and quality report. | XL | High | 10/10 if premises hold |
| C known/reviewed map localization or LIO-derived occupancy/offline-first | Use reviewed static maps for competition navigation or derive occupancy from LIO/offline processing first. | L | Medium | 8/10 for competition operation |

Recommendation: preserve B as the target but add an evidence gate before implementing the tracker. This is `USER CHALLENGE C1` because it changes user-approved stage ordering; the original direction remains pending final gate.

### 0D Scope and selective expansion

Original scope baseline: implement the SLAM dynamic obstacle filter design from `docs/superpowers/specs/2026-07-19-slam-dynamic-obstacle-filter-design.md`, preserving online SLAM filtering, Nav2 no-delay avoidance, safe candidate map saving, deterministic offline replay and R5 CPU-only constraints.

Auto-decisions included because they directly harden the blast radius:

- No shell map saving: replace `auto_save_map.launch.py:15-27` shell path with typed `nav2_map_server` client semantics.
- Session IDs and fixed output root: every bag, candidate map, diagnostics CSV/JSON and metadata file share one generated session identity and canonical path containment.
- Diagnostic freshness: stale scan, stale odom, stale mask, stale recorder or stale disk health transitions to DEGRADED/FROZEN and blocks save.
- Bounded queues: scan, TF, tracker, mask and offline replay buffers have explicit max sizes and overflow result codes.
- Rollback mode: launch arg supports `disabled|observe|enforce`, with `observe` as the default rollout bridge.
- Whole-system performance metrics: include control frequency jitter, cmd_vel latency, TF latency, total CPU/RSS and thermal throttling across 2h.

Deferred by scope:

- Camera/semantic network: Realsense source is configured but stale and launch is commented out at `multiplenav_launch.py:155-164`; adding semantic classification would not prove lidar-only map cleanliness.
- Cloud processing: no internet, no secrets, no remote dashboard needed; reproducibility must be local.
- Cross-session classifier: accepted limitation says session memory only; permanent object identity would change the product.
- Replacing `slam_toolbox` or `small_point_lio`: not needed for Phase 1 evidence and would expand integration risk.
- UI product work: operator-facing diagnostics and RViz markers are enough for this robotics workflow.

### 0E Temporal interrogation

| Time window | Human review would encounter | CC compressed-time review result |
|---|---|---|
| Hour 1 | Package/API boundaries and state machine: where to insert filter, what services exist, whether `safe_map_saver` is separate. | Independent package is justified; pure library plus thin ROS nodes; state machine must reject invalid save/reset transitions. |
| Hours 2-3 | Scan stability and same-source odom coupling: Livox projection fidelity, `time_increment=0`, 45-degree install, LIO consuming MID-360 point cloud + built-in IMU. | C1 gate required before tracker implementation; Gazebo validates algorithm/state logic only and cannot close real odom accuracy because its model is 360 single-layer 10Hz. |
| Hours 4-5 | Launch/QoS/session/lifecycle integration: disabled/observe/enforce, diagnostics freshness, session IDs, map saver lifecycle. | Add explicit launch modes, bounded SensorDataQoS paths, session-matched save requests, HEALTHY-only save state. |
| Hours 6+ | Fault injection, map metrics, thermal profile and docs: bag corruption, disk full, map service timeout, 2h R5 run, runbooks. | Error registry and test pyramid cover runtime branches; whole-system performance becomes a Phase 1 acceptance item. |

### CEO outside voices

Claude subagent was unavailable because the environment model configuration referenced `deepseek-v4-flash`, which is unavailable. Codex ran the primary code review. There is no dual-model confirmation, and no Claude subagent agreement is claimed.

Codex critical/high findings:

- Unstable 2D abstraction: projected MID-360 is treated as repeatable planar lidar, but `pointcloud_to_laserscan_node.cpp:150` sets `time_increment = 0.0` and Gazebo is explicitly a single-ring approximation in `docs/livox-debug-guide.md:72-80`.
- Same-source odom: `small_point_lio_node.cpp:197-210` uses the MID-360 point cloud + built-in IMU before publishing `/Odometry`, so ego-motion compensation is not independently validated by wheel/electrical IMU hardware.
- Map cleanliness vs safety: STVL has a no-delay path from `/livox/lidar_filtered` in `multiplenav2_params.yaml:212-229`, but freshness stop/collision monitor is absent while speeds reach 2.5 m/s at `:93-99` and `:470-475`.
- Two mapping stories: online `slam_toolbox` map and offline replay/refinement must share frozen metadata or candidate maps will be hard to compare.
- Gazebo fidelity: simulation launch wires pointcloud projection and SLAM at `gazebo_slam.launch.py:94-110`, but docs warn it is not real MID-360 pattern; Gazebo validates algorithm/state logic only, not real odom accuracy.
- Session identity overconfidence: current autosave has no session ID and shells into a timestamp-like suffix in `auto_save_map.launch.py:15-27`.
- Alternatives/staging: evidence-first A lowers risk; approved B remains target if C1 passes.
- Whole-system budget: Nav2 MPPI 50Hz, STVL 20Hz and velocity smoother 30Hz coexist with SLAM/filter/bag IO in `multiplenav2_params.yaml:60-87`、`:172-176`、`:465-475`.
- Realsense stale config: `realsense_source enabled: true` in params `multiplenav2_params.yaml:235-256` while launch comments out the camera at `multiplenav_launch.py:155-164`.

| Dimension | Claude | Codex | Consensus |
|---|---|---|---|
| 1 Architecture | N/A | Critical concerns verified; coupling otherwise justified | N/A |
| 2 Error/rescue | N/A | High: explicit result categories needed | N/A |
| 3 Security | N/A | High: shell saver/path/session/disk mitigations required | N/A |
| 4 Data flow | N/A | High: stale/empty/TF/order edge cases mapped | N/A |
| 5 Performance | N/A | High: whole-system budget missing | N/A |
| 6 Rollout | N/A | High: observe/enforce staging required | N/A |

Primary code review independently verified the critical facts above from repository files and line references.

### Section 1 Architecture

Happy path and shadow paths:

```text
REAL INPUT
  /livox/lidar PointCloud2
    -> cpp_lidar_filter
       -> /livox/lidar_filtered
          -> Nav2 STVL local/global costmaps      [no-delay safety branch]
          -> pointcloud_to_laserscan measured path
             -> /scan_raw
                -> dynamic_obstacle_filter_node
                   + /Odometry + TF
                   + SessionHealth
                   -> /scan_slam_filtered
                      -> slam_toolbox
                         -> /map
                            -> safe_map_saver typed service
                               -> candidate map + metadata + report

SHADOW PATHS
  observe mode:
    /scan_raw -> filter compute -> mask/diagnostics only
    /scan_raw continues to slam_toolbox for baseline; final save blocked
  disabled mode:
    existing stack may run; output is labeled unsafe for final maps
  offline mode:
    bag(/scan_raw,/tf,/Odometry,diag,params,git) -> deterministic replay -> candidate map/report
  rollback:
    enforce -> observe -> disabled without deleting bags or candidate maps
```

State machine:

```text
STARTING -> WARMING -> HEALTHY -> DEGRADED/FROZEN -> ERROR
              ^          |              |
              |          +--------------+
reset/restart creates a new session_id and returns to WARMING
save is allowed only in HEALTHY with fresh diagnostics and matching session_id
invalid transitions are rejected with typed result codes and diagnostics
```

Findings:

- Unproven scan abstraction: launch config implies about 720 beams with `angle_increment: 0.0087` at `multiplenav_launch.py:74-77`, but actual converter gives zero per-beam timing at `pointcloud_to_laserscan_node.cpp:147-151`.
- Same-source odom: LIO publishes `/Odometry` after consuming MID-360 point cloud + built-in IMU at `small_point_lio_node.cpp:197-210`; use as candidate only until C1 evidence passes, and if evidence fails remain observe and never enforce.
- Missing Nav2 safety gate: STVL is present, but `expected_update_rate: 0.0` at `multiplenav2_params.yaml:225` and `:315` plus no `collision_monitor` match means stale sensor stop is not configured.
- Rollback mode is required: launch must expose `disabled|observe|enforce`; `observe` and `disabled` preserve operations while preventing unsafe final map promotion.
- Original design coupling is otherwise justified: filtering before `slam_toolbox` directly addresses the verified `/scan` pollution path, while STVL remains the immediate avoidance branch.

### Section 2 Error & Rescue Registry

| Method / codepath | Result category | Rescue behavior | User-visible behavior |
|---|---|---|---|
| `ScanIngress::onScan()` receiving no scan before warmup deadline | `ScanStatus::STALE_INPUT` | Stay WARMING, publish stale diagnostic, block save. | `/diagnostics` WARN `scan_stale`; save service returns `FILTER_NOT_HEALTHY`. |
| `ScanIngress::onScan()` TF lookup missing | `ScanStatus::TF_MISSING` | Drop frame, increment TF failure rate, enter DEGRADED if threshold exceeded. | Throttled ERROR with source/target frame; diagnostic `tf_failure_rate`. |
| `ScanIngress::onScan()` TF extrapolation | `ScanStatus::TF_EXTRAPOLATION` | Keep previous HEALTHY only within freshness window; otherwise FROZEN. | WARN `tf_extrapolation`; RViz marker color changes for filter state. |
| `ScanValidator::validate()` invalid scan geometry | `ScanStatus::INVALID_GEOMETRY` | Reject scan, do not mutate tracker memory. | Service/report counts invalid angle/range fields. |
| `ScanValidator::validate()` all `inf` or all `NaN` | `ScanStatus::EMPTY_SCAN` | Publish empty/unchanged mask, mark insufficient evidence. | Diagnostic `dynamic_ratio=0`, `valid_beam_count=0`; save blocked if sustained. |
| `BoundedScanQueue::push()` overflow | `QueueStatus::OVERFLOW` | Drop oldest or newest by configured policy, enter DEGRADED/FROZEN. | WARN `scan_queue_overflow`, count exported. |
| `Tracker::associate()` too many candidates | `TrackStatus::OVER_CAPACITY` | Cap active tracks, skip new dynamic promotions, block final save. | Diagnostic ERROR `track_capacity_exceeded`. |
| `Tracker::associate()` ambiguous association | `TrackStatus::AMBIGUOUS_ASSOCIATION` | Keep object unconfirmed; do not mask as dynamic until stable. | DEBUG/WARN count `ambiguous_tracks`. |
| `HealthPublisher::tick()` heartbeat stale | `HealthStatus::STALE_DIAGNOSTIC` | Saver denies; launch monitor may freeze SLAM input. | `/diagnostics` ERROR and save response names stale key. |
| `SafeMapSaver::save()` path outside root | `SaveStatus::PATH_OUTSIDE_ROOT` | Reject request, no filesystem write. | Service response `success=false`, reason logged with canonical path. |
| `SafeMapSaver::save()` session mismatch | `SaveStatus::SESSION_MISMATCH` | Reject request, keep candidate maps untouched. | Service response includes requested/current session IDs. |
| `SafeMapSaver::save()` disk reserve low | `SaveStatus::LOW_DISK_RESERVE` | Reject before map service call. | Diagnostic `disk_free_bytes`; save denial reason. |
| `SafeMapSaver::save()` map service timeout | `SaveStatus::MAP_SERVICE_TIMEOUT` | Cancel/return false; no final promotion. | ERROR log with timeout and lifecycle state. |
| `SafeMapSaver::save()` map service returns false | `SaveStatus::MAP_SERVICE_FAILED` | Preserve request metadata and failure report. | Service response carries map_server message. |
| `MappingBagRecorder` process exits unexpectedly | `RecorderStatus::PROCESS_EXITED` | Mark session not reproducible; block final save. | Diagnostic ERROR and recorder log path. |
| `MappingBagRecorder` disk reserve failure | `RecorderStatus::LOW_DISK` | Stop recording cleanly if possible; freeze final save. | Operator sees bag/disk WARN before ERROR. |
| `OfflineReplay::loadMetadata()` missing metadata | `ReplayStatus::METADATA_MISSING` | Abort replay, leave source bag untouched. | CLI/report says missing file and path. |
| `OfflineReplay::deserialize()` corrupt message | `ReplayStatus::DESERIALIZATION_FAILED` | Abort current run, retain failure report. | Report names topic, timestamp, message type. |
| `OfflineReplay::sort()` timestamp/order mismatch | `ReplayStatus::ORDER_MISMATCH` | Deterministic stable sort if allowed, otherwise abort. | Report shows first offending timestamps. |
| `OfflineSlamRunner` timeout | `ReplayStatus::SLAM_TIMEOUT` | Kill child process without shell, mark candidate invalid. | CLI exit nonzero and JSON report timeout field. |
| Launch node death for filter/saver/recorder | `LaunchStatus::NODE_EXITED` | Lifecycle monitor transitions ERROR or rollback mode. | Launch log and diagnostic `node_alive=false`. |

### Section 3 Security & Threat Model

Threat boundary: ROS service callers are assumed to be on a trusted DDS network. This must be documented because DDS discovery and service calls are not an authentication boundary in this repo.

| Threat | Current evidence | Decision / mitigation |
|---|---|---|
| Reset/freeze/save abuse by any ROS caller | No service auth layer exists in current launch files. | Treat as trusted-network operation; document DDS trust boundary; make destructive operations session-scoped and observable. |
| Path traversal via map output | Current autosave constructs shell command with `save_dir` and suffix at `auto_save_map.launch.py:15-27`. | Fixed root, generated session IDs, canonical path containment, reject `..`, absolute user paths and symlink escape. |
| Command injection | Current saver invokes `bash -c` at `auto_save_map.launch.py:16-19`. | No `bash -c`; use typed ROS service clients and direct filesystem APIs. |
| Bag disk exhaustion | `record_bag.sh` disables size splitting with `--max-bag-size 0` at `scripts/record_bag.sh:181-184`. | Add disk reserve checks before and during mapping recorder, bounded duration/size arguments, visible LOW_DISK state. |
| Malicious/corrupt bag | Offline replay will ingest serialized ROS messages. | Validate metadata, topic allowlist, message type, max message size, monotonic/stable order, and fail closed on deserialization errors. |

No internet, PII or secrets are part of this design. Metadata should include git revision and parameters, not credentials.

### Section 4 Data Flow and Edge Cases

```text
INPUT -> VALIDATE -> COMPENSATE -> BUFFER -> SEGMENT -> TRACK -> MASK -> PUBLISH
  |        |           |            |         |         |        |       |
  nil      invalid     TF/odom      overflow  empty     ambiguous NaN    absent subscriber
  stale    all inf     jump         stale     high dyn  over cap  deny   diagnostics only
  corrupt  NaN         reversed ts  frozen    ratio     reset     error  no crash
```

Edge cases and decisions:

- Empty scan: publish diagnostic and no dynamic promotion; save blocked if sustained.
- All `inf`: valid no-return frame, but insufficient map evidence if repeated.
- NaN ranges: preserve semantic for masked/unknown beams and reject non-finite geometry metadata.
- Reversed timestamps: reject online frame or reorder only in offline stable-sort mode with report.
- TF extrapolation: drop frame and count; freeze if rate crosses threshold.
- Odom jump: reset tracker session memory or move to DEGRADED; never compensate across discontinuity silently.
- Dynamic occupancy ratio too high: publish FROZEN and stop masking rather than erase too much map evidence.
- Duplicate save calls: idempotent per session/request ID; concurrent save returns `SAVE_IN_PROGRESS`.
- Stale health: safe saver denies even if `slam_toolbox` map service is available.
- Session mismatch: reject save/reset/final promotion; current session remains intact.
- Partial bag: offline replay fails closed and emits report, preserving source bag.
- Output subscriber absent: compute diagnostics if configured; skip expensive marker publication where no subscriber, but do not alter state semantics.

### Section 5 Code Quality

An independent package is justified because the feature spans online filtering, diagnostics, saver gates and offline replay without belonging cleanly inside `cpp_lidar_filter`, `pointcloud_to_laserscan`, `slam_toolbox` or Nav2. Keep pure library modules and thin ROS nodes:

- Pure library: scan validation, odom interpolation, tracker, mask generation, session health, metadata writer, deterministic replay core.
- ROS nodes: online filter node, diagnostics publisher, mapping recorder wrapper, safe map saver executable, offline CLI/launch wrapper.
- `safe_map_saver` may live in the same package but should be a separate executable because its dependencies and failure modes differ from scan processing.
- No custom message v1: use standard diagnostics, parameters, services and JSON/CSV reports first; introduce custom interfaces only after the schema stabilizes.
- Central shared `SessionHealth` and `FilterConfig` types avoid drift between online/offline code.
- Use explicit enums/result types such as `ScanStatus`, `TrackStatus`, `SaveStatus`, `ReplayStatus`; avoid Boolean-only failure returns.
- Keep functions branch count <=5 by decomposition: validate, compensate, associate, mask and publish should be separate units.
- Avoid duplicated online/offline tracking: offline replay should call the same tracker/mask library with deterministic time inputs.

### Section 6 Test Review

Test pyramid:

```text
many: pure unit tests for geometry, time, state, metadata
some: ROS component/integration tests for topics, TF, services, lifecycle
few: launch/Gazebo/bag-replay/perf/chaos tests
required gate: real-bag Stage 0 feasibility tests if C1 is accepted
```

| Flow | Unit | Integration / launch | Gazebo / bag / perf |
|---|---|---|---|
| Data: empty/all-inf/NaN/range/angle validation | `ScanValidator` parameterized cases | ROS scan fixture publishes invalid and valid scans | Bag replay includes empty/static/dynamic clips |
| Data: TF interpolation/extrapolation | deterministic TF buffer tests | tf2 missing/stale integration | Gazebo TF delay injection |
| Data: same-source odom coupling metrics | odom delta and jump detector | compare `/Odometry` and TF continuity | Real bag Stage 0 static-wall residual, LIO continuity/drift before/after dynamic targets, and compensated static consistency report |
| Code: segmentation/tracking/association | cluster split/merge/ambiguous tests | observe mode publishes masks/diagnostics | Moving obstacle bag replay |
| Code: bounded queues | overflow policy tests | high-rate scan publisher triggers DEGRADED | perf test records dropped rate |
| Error: stale heartbeat/save denial | state machine tests | safe saver service tests for stale/session mismatch | launch test kills diagnostic node |
| Error: disk/path/map timeout | path canonicalization and disk mock | fake map service timeout/false result | chaos test fills temp root reserve |
| Background: recorder lifecycle | argument builder and metadata tests | process exit detection | bag completeness check |
| Offline: metadata/deserialization/order | parser and stable sort tests | offline CLI exits with named code | corrupt/partial bag replay |
| Integration: modes | config parser tests | `disabled|observe|enforce` launch tests | smoke checks after rollout |
| Performance: R5 constraints | microbench process time tests | component p95 latency under load | 2h total CPU/RSS/temp/frequency throttling |

Chaos tests include node death, TF outage, map service hang, bag writer exit, low disk reserve, scan burst overflow, odom discontinuity and duplicate save calls. Every branch from the design must either publish diagnostics, deny save, rollback mode or emit a structured replay report. Real-bag Stage 0 feasibility tests are missing from the approved staging and are subject to `USER CHALLENGE C1`.

### Section 7 Performance

Top slow paths:

- PointCloud2 projection to LaserScan: current projection iterates all cloud points and picks nearest beam at `pointcloud_to_laserscan_node.cpp:178-228`.
- Odom compensation over buffered scans: repeated TF/odom interpolation for 0.8-1.2s windows.
- Segmentation/association/tracking: nearest-neighbor or cluster association can grow with object/beam count.
- Mask publication and diagnostics serialization: bounded but must avoid per-frame heap churn.
- Bag recording and offline SLAM: write bandwidth and CPU compete with online Nav2 if run simultaneously.

Bounded sizes:

- Fixed scan history by time and max frames.
- Fixed max beams from measured scan geometry, rejecting changes outside tolerance.
- Fixed max tracks and candidate associations.
- Fixed map save concurrency of one active request.
- Fixed recorder disk reserve and max bag duration/size for mapping sessions.

Whole-system metrics added:

- Total CPU and per-process CPU for filter, SLAM, Nav2, recorder and LIO.
- Control frequency jitter for controller 50Hz and velocity smoother 30Hz from `multiplenav2_params.yaml:60-87`、`:465-475`.
- `cmd_vel` to `/aft_cmd_vel` latency, including `fake_vel_transform` and serial remap from `multiplenav_launch.py:134-153`.
- Cloud/scan dropped rate and queue overflow counts.
- TF lookup latency and extrapolation failure rate.
- Temperature/frequency throttling over 2h on R5-4500U.
- RSS for each process plus total resident memory.

Existing per-process criteria are retained: average filter load <=50% of one core, short peaks only, <200 MB extra RSS, p95 frame processing <30 ms, p95 SLAM delay 0.8-1.3s, Nav2 avoidance added latency <10 ms, no leaks/crashes over 2h and offline capped to two cores.

### Section 8 Observability

Diagnostics keys/status codes:

- `session_id`, `mode`, `state`, `state_since`, `config_hash`, `git_revision`.
- `scan_status`, `valid_beam_count`, `beam_count`, `scan_rate_hz`, `scan_age_ms`, `time_increment_zero`.
- `odom_status`, `odom_age_ms`, `odom_jump_count`, `odom_source=/Odometry`.
- `tf_status`, `tf_lookup_latency_ms`, `tf_failure_rate`, `tf_extrapolation_count`.
- `tracker_status`, `active_tracks`, `ambiguous_tracks`, `track_capacity`, `dynamic_beam_ratio`.
- `queue_depth_scan`, `queue_depth_track`, `dropped_scan_count`, `mask_latency_ms`.
- `save_status`, `save_denial_reason`, `last_save_request_id`, `map_service_latency_ms`.
- `bag_status`, `bag_path`, `bag_bytes`, `disk_free_bytes`, `disk_reserve_ok`.
- `cpu_total_pct`, `rss_mb`, `temperature_c`, `throttled_frequency`.

Logs should be structured and throttled, with high-cardinality values in JSON/CSV reports rather than unbounded console spam. Operator runbook must list what to do for WARMING, DEGRADED/FROZEN, ERROR, save denied, low disk and stale sensor. RViz markers should show crop box, dynamic mask and state color. No remote dashboard is required.

### Section 9 Deployment & Rollout

Launch arg modes:

- `disabled`: dynamic filter not used. Existing baseline may run, but any output map is labeled unsafe for final maps and final save is blocked by policy.
- `observe`: filter computes diagnostics and masks, publishes reports/markers, but SLAM remains on current input only for baseline comparison; final save is blocked.
- `enforce`: SLAM consumes filtered scan, saver allows candidate maps only when HEALTHY and session-matched.

Rollout:

1. Real bag offline feasibility: measure projected scan stability, same-source `/Odometry` usability and dynamic/static separability using static-wall residual, LIO continuity/drift before and after dynamic targets, and compensated static consistency during straight/lateral/rotation motions.
2. Observe on robot at low speed: verify diagnostics, masks, CPU/RSS, TF, recorder and save denial.
3. Low-speed enforce: route `/scan_slam_filtered` into `slam_toolbox`; compare candidate maps.
4. Full mapping speed: enable only after freshness stop/collision gate and 2h thermal/control profile pass.
5. Candidate review and manual final promotion: reviewed static maps become competition navigation maps.

Rollback: switch enforce -> observe -> disabled without deleting bags, reports or candidate maps. Candidate-only saves remain available for forensic comparison, but final promotion remains gated. Post-deploy smoke checks include topic graph, diagnostics fresh, STVL receiving `/livox/lidar_filtered`, `slam_toolbox` receiving expected scan, saver denies stale health, recorder writes metadata, and rollback preserves session artifacts.

### Section 10 Long-term

Reversibility: 4/5. The design is reversible if launch modes, topic remaps and candidate-only saves are kept explicit. The risky long-term debt is a bespoke 2D tracker and metadata protocol; shared pure libraries, explicit enums and versioned metadata reduce that debt.

A/B/C trajectory:

- A provides evidence and map-safety hardening with low implementation load; it is the best immediate de-risk step.
- B remains the approved target if C1 passes; it provides the cleanest live simultaneous-SLAM map and offline refinement story.
- C may become the operational winner if reviewed static maps and runtime costmaps are enough for competition, even if it does not provide clean live simultaneous-SLAM mapping.

Six-month ideal question: does the online tracker still earn its complexity compared with a reviewed-map workflow plus runtime STVL/collision gating? Keep the evidence reports good enough to answer that with data rather than preference.

### Section 11 Design

Skipped because there is no UI surface in this feature. Operator-facing diagnostics, RViz markers, service responses, logs, CSV/JSON reports and runbook coverage are handled by DX/observability in Sections 8 and 9.

### Error and Failure Modes Registry

| Codepath | Failure | Rescued | Test | Operator sees / logged |
|---|---|---|---|---|
| `ScanIngress::onScan` | Missing input topic or no messages | WARMING then DEGRADED; save blocked | launch test without publisher | `/diagnostics` WARN/ERROR `scan_stale` |
| `ScanValidator` | Invalid angle/range geometry | Frame rejected; tracker unchanged | unit invalid geometry | throttled ERROR and invalid count |
| `ScanValidator` | Empty/all-inf/all-NaN frame | No dynamic promotion; health degraded if sustained | unit + bag fixture | `valid_beam_count`, `empty_scan_count` |
| `TfOdomBuffer` | Missing TF | Drop frame; count failure | integration missing TF | `tf_missing` with frame names |
| `TfOdomBuffer` | TF extrapolation | Drop or freeze by freshness threshold | delayed TF test | `tf_extrapolation_count` |
| `OdomMonitor` | Odom jump | Reset tracker memory or DEGRADED | odom discontinuity unit/integration | `odom_jump_count`, state transition |
| `BoundedScanQueue` | Overflow | Apply configured drop policy; FROZEN if repeated | burst publisher test | `scan_queue_overflow` |
| `Segmenter` | Dynamic ratio too high | Stop masking and deny save | synthetic high dynamic ratio | `dynamic_ratio_high` |
| `Tracker` | Ambiguous association | Keep unconfirmed; no mask | crossing tracks unit test | `ambiguous_tracks` |
| `Tracker` | Track capacity exceeded | Cap tracks; DEGRADED | capacity unit/perf test | `track_capacity_exceeded` |
| `MaskPublisher` | Output subscriber absent | Publish diagnostics; skip optional heavy markers | integration no subscriber | DEBUG count, no crash |
| `HealthPublisher` | Diagnostic heartbeat stale | Saver denies; state FROZEN | heartbeat stop test | save denial `STALE_DIAGNOSTIC` |
| `SafeMapSaver` | Duplicate save call | Return `SAVE_IN_PROGRESS` or same completed result | concurrent service test | service response with request ID |
| `SafeMapSaver` | Session mismatch | Reject; no write | service unit/integration | `SESSION_MISMATCH` |
| `SafeMapSaver` | Path traversal/root escape | Reject before filesystem write | canonical path unit | `PATH_OUTSIDE_ROOT` |
| `SafeMapSaver` | Disk reserve low | Reject before map service | disk mock | `LOW_DISK_RESERVE` |
| `SafeMapSaver` | Map service timeout/false | Fail closed; candidate invalid | fake service timeout/false | `MAP_SERVICE_TIMEOUT` / `MAP_SERVICE_FAILED` |
| `Recorder` | Process exits | Mark session not reproducible; block final | killed process launch test | `RECORDER_EXITED`, log path |
| `Recorder` | Partial bag metadata absent | Offline replay aborts | partial bag test | `METADATA_MISSING` |
| `OfflineReplay` | Deserialization/order mismatch | Abort or stable-sort with report by configured policy | corrupt/order bag tests | JSON report topic/timestamp |
| `OfflineSlamRunner` | SLAM timeout | Stop child process; candidate invalid | timeout test | nonzero CLI and report field |
| Launch monitor | Filter/saver node death | ERROR or rollback mode | launch kill test | `node_alive=false` |

After proposed hardening there are no silent runtime rescue gaps: every listed branch either blocks saving, rolls back/freezes, rejects the request or emits a structured report. `USER CHALLENGE C1` remains an unresolved feasibility gate, not a runtime rescue gap.

### Diagrams and stale diagram audit

Required diagrams for the engineering phase:

- System diagram: raw Livox, crop filter, pointcloud projection, dynamic filter, SLAM, STVL, saver and recorder.
- Data-flow diagram: INPUT -> VALIDATE -> COMPENSATE -> BUFFER -> SEGMENT -> TRACK -> MASK -> PUBLISH plus nil/empty/error/stale shadow paths.
- State diagram: STARTING -> WARMING -> HEALTHY -> DEGRADED/FROZEN -> ERROR and reset/restart behavior.
- Error diagram: failures to diagnostics/save denial/rollback/report.
- Deploy diagram: disabled -> observe -> enforce -> rollback.
- Offline replay diagram: bag + frozen metadata -> deterministic tracker -> offline SLAM -> candidate map/report.

Docs/architecture diagrams that must be updated if implementation proceeds:

- `docs/QUICK_START.md:101-113` currently shows `/livox/lidar -> pointcloud_to_laserscan -> /scan -> slam_toolbox`; it must show `/scan_raw` and `/scan_slam_filtered`.
- `cod_-rm2026_-navigation-master/src/cod_gazebo_simulator/README.md:94-100` currently describes sim `/livox/lidar -> /scan`; it must label Gazebo as secondary and add observe/enforce modes if used in docs.
- `docs/livox-debug-guide.md:72-80` already warns about Gazebo fidelity; keep that warning linked from rollout docs.
- Launch files currently contain comments, but no relevant dynamic-filter ASCII diagram exists in launch files to go stale.

### Workstream dependency and rollout

| Lane | When | Dependencies | Conflicts / notes |
|---|---|---|---|
| Feasibility / recording first | First if C1 approved | Mapping-specific recorder; real bags with `/livox/lidar`, `/scan_raw`, `/Odometry`, TF, params, git metadata; metrics scripts. | Changes approved ordering; does not implement tracker until evidence passes. |
| Online library | After C1 or in parallel only as pure prototypes | Stable scan schema, odom confidence, bounded queue policy, state/result enums. | Avoid coupling tracker directly to ROS node so offline can reuse it. |
| Saver/session | Early hardening | `SessionHealth`, fixed output root, map service client, disk checks, diagnostics. | Must replace `auto_save_map.launch.py` unconditional shell semantics. |
| Launch/sim | After mode API exists | `disabled|observe|enforce` arg, remaps, lifecycle monitor, Gazebo smoke worlds. | Gazebo validates algorithm/state logic only, not MID-360 fidelity or real odom accuracy. |
| Later real calibration | After observe metrics | Real static/dynamic/multi-person/robot bags and R5 thermal profiles. | Calibration may force tracker thresholds or reject B premise. |
| Later offline refinement | After shared online library and metadata freeze | Deterministic replay, offline SLAM wrapper, quality reports and reproducibility test. | Must not duplicate online tracking logic. |

Dependencies: saver/session can start before tracker because it reduces existing map corruption risk. Feasibility recording must precede enforce rollout if C1 is accepted. Launch/sim cannot be final acceptance because real sensor fidelity is unresolved. Offline refinement depends on stable metadata and shared library interfaces.

### CEO Completion Summary

| Item | Result |
|---|---|
| Mode | SELECTIVE_EXPANSION |
| UI / Design | Section 11 skipped; no UI, operator diagnostics covered |
| Architecture concerns | 4 concerns: unproven scan abstraction, same-source odom coupling, missing Nav2 safety gate, rollback mode |
| Error paths | Mapped with named result categories and user-visible rescue behavior |
| Security | 5 threats mitigated: ROS caller abuse boundary, path traversal, command injection, bag disk exhaustion, malicious/corrupt bag |
| Data edge cases | 10+ covered: empty scan, all inf, NaN, reversed timestamps, TF extrapolation, odom jump, high dynamic ratio, duplicate save, stale health, session mismatch, partial bag, absent subscriber |
| Tests | Test plan complete across unit/integration/launch/Gazebo/bag-replay/perf/chaos; C1 feasibility ordering unresolved |
| Performance | 1 whole-system gap added beyond per-process filter budget |
| Observability/deploy | Diagnostics, logs, CSV/JSON reports, runbook, modes and rollout hardening included |
| Accepted unrelated expansions | 0 |
| User Challenges | 1: C1 evidence gate before tracker implementation |

Phase 1 complete. Pre-Phase-2 checklist:

- Decide `USER CHALLENGE C1`: approve evidence-first Stage 0 or proceed with original Stage A ordering knowingly.
- If C1 is approved, define real-bag metrics for projected scan stability and same-source odom usability: static-wall residual, LIO continuity/drift before and after dynamic targets, and compensated static consistency during straight/lateral/rotation motions.
- Freeze package/API names, launch modes and state/result enums before Eng.
- Treat Gazebo as algorithm/state logic smoke only; Gazebo ground truth does not validate real odom accuracy.

### User Challenges pending final gate

| ID | Challenge | Why it needs user decision | Default if rejected |
|---|---|---|---|
| C1 | Add an evidence gate before implementing the online tracker: record real bags, measure projected Livox scan stability, validate same-source `/Odometry` usability with static-wall residual, LIO continuity/drift before and after dynamic targets, compensated static consistency during straight/lateral/rotation motions, and establish whole-system R5 thermal/control budget. Only MID-360 built-in IMU is available; electrical-control IMU is not connected; no independent wheel odometry is available. | This changes the approved stage ordering by moving feasibility evidence before tracker implementation, even though it preserves the approved full B architecture as the target. `/Odometry` is same-source because `small_point_lio` uses MID-360 point cloud + built-in IMU; Gazebo ground truth validates algorithm/state logic only, not real odom accuracy. | Continue original Stage A online+offline implementation order in observe-only validation; if evidence fails, remain observe and never enforce. Independent wheel/electrical IMU is future NOT-in-scope hardware only. |

End of Phase 1 CEO Review.

## Phase 3 — Engineering Review

### Scope Challenge

基于 Phase 1 已核实的代码事实，这个设计触及超过 8 个文件、超过 2 个组件是合理的，但只合理为三个顺序 deliverables，而不是一个 PR：

1. Deliverable 1: 最小安全 hardening。范围是 saver、session、recording、observe mode、typed state、metadata 和证据采集，不实现动态 tracker enforce。
2. Deliverable 2: online filter core。范围是 validator、compensator、segmenter、tracker、masker、filter node、Nav2 freshness/collision safety 和 launch observe/enforce 切换。
3. Deliverable 3: offline replay/refinement。范围是 deterministic replay、offline refiner、SLAM rerun、quality report、schema migration 和 final promotion gate。

Minimum safe hardening 是 saver/session/recording/observe mode。完整目标仍然保留：online SLAM filtering、session memory、safe candidate map saving、deterministic offline replay/refinement、R5 CPU-only 验收。现有 component reuse map 继承 Phase 1：`cpp_lidar_filter` 继续做 crop/voxel，`pointcloud_to_laserscan` 继续做投影但必须量测，`small_point_lio` 仅作为 same-source candidate odom，STVL 保持 no-delay branch，`slam_toolbox` 和 `nav2_map_server` 复用，`auto_save_map` 的 shell 行为删除或 disabled。当前硬件只有 MID-360 built-in IMU；电控 IMU 未连接；无独立 wheel odometry。

仓库中没有可复用的 TODO file；现有 `package.xml` 中的 TODO license 属于无关技术债，不复制到新包。发行形态是 ROS workspace 内通过 `colcon` 构建和安装，不是外部 package publishing；因此必须补齐 `install()` targets、launch/config/scripts 安装、`package.xml` runtime/test dependencies 和 dependency docs。

搜索层策略：

```text
Layer 1: laser_filters temporal median baseline
  先用已有 ROS 生态的 temporal median / scan filter 思路建立低复杂度 baseline 和真实 bag 指标。

Layer 2: research cluster/track filters
  再对比 clustering、multi-frame persistence、track confirmation、occlusion/reactivation 等研究型策略。

Layer 3: custom tracker only after evidence
  moving-robot Livox projection 和 same-source LIO odom 都需要证据；`small_point_lio` `/Odometry` 使用 MID-360 point cloud + built-in IMU，不是独立 wheel/electrical IMU。只有 C1 真实 bag 指标证明 beam persistence、scan coverage、static-wall residual、动态目标前后 LIO continuity/drift、以及直行/横移/旋转中的 compensated static consistency 后，才实现 bespoke tracker enforce；若证据失败，保持 observe，绝不 enforce。
```

C1 仍是唯一 scope/order User Challenge：是否把 evidence gate 放到 tracker 实现之前。不得把 C1 标为 accepted。

### Engineering outside voices

Claude subagent 不可用，原因是环境配置引用 `deepseek-v4-flash`。Codex 已运行并完成代码审查。没有 Claude 双确认，也不声明 Claude consensus。

Codex 输出的 10 个工程发现：

1. Evidence gate 不能作为可选项；Livox projection 和 same-source LIO odom 前提未被代码证明。
2. 当前 bag 缺 raw/filtered cloud、实际 MID-360 built-in IMU topic、driver config、scan、mask、diagnostics、TF、odom 的完整输入链，无法做 deterministic replay。
3. `time_increment = 0` 且 scan repeatability 未证明，不能直接把 projection 当稳定 planar lidar。
4. Same-source odom/twist coupling 缺证据；`small_point_lio` 使用 MID-360 point cloud + built-in IMU，不能默认当作独立 wheel/electrical IMU 对比。
5. Nav2 safety 缺 freshness/collision gate；STVL no-delay branch 存在但 freshness 停车语义不足。
6. Gazebo sensor model 与 MID-360 真实 pattern 不匹配，只能 smoke。
7. Deterministic replay contract 缺稳定排序、时间、配置、schema 和 tolerance 定义。
8. Health/session APIs 如果使用 free-form diagnostics 会脆弱；saver 必须使用 typed state/request。
9. NaN/inf 对 SLAM 的实际行为未证明，必须做集成测试。
10. Security acceptance 必须直接写成准入条件：无 shell、canonical root、symlink containment、bounded sizes、topic allowlist、one active save、disk reserve、typed IDs、trusted DDS boundary。

| Dimension | Claude | Codex concern | Consensus |
|---|---|---|---|
| 1 Architecture | N/A | Evidence gate、typed services、pure core/thin ROS adapters、STVL direct branch 和 offline shared library 必须锁定 | N/A |
| 2 Error/rescue | N/A | 每个错误必须有 enum、test、diagnostic/log、recovery；free-form health 不足 | N/A |
| 3 Security | N/A | shell saver、path traversal、symlink escape、duplicate save、disk exhaustion、bag deserialization 必须 fail closed | N/A |
| 4 Data flow | N/A | raw/filtered cloud、IMU、scan、mask、diag、TF、odom 和 frozen metadata 必须全链记录 | N/A |
| 5 Performance | N/A | 单进程预算不够，必须加入整机 CPU、control jitter、TF、thermal、RSS 和 bag IO | N/A |
| 6 Rollout | N/A | disabled -> observe -> enforce；C1 未过不得 enforce；Gazebo 只 smoke | N/A |

Primary code audit verified facts：现有 SLAM 输入链、STVL 分支、`auto_save_map` shell、bag topic 缺口、Gazebo 近似、`time_increment=0`、LIO 同源 odom 和 Nav2 freshness 缺口均来自仓库代码/配置，不是推测。用户确认当前硬件边界：只有 MID-360 built-in IMU，电控 IMU 未连接，且无独立 wheel odometry。

### Architecture

锁定新增 package 路径：

```text
cod_-rm2026_-navigation-master/src/slam_dynamic_filter/
  package.xml
  CMakeLists.txt
  include/slam_dynamic_filter/types.hpp
  include/slam_dynamic_filter/scan_validator.hpp
  include/slam_dynamic_filter/motion_compensator.hpp
  include/slam_dynamic_filter/scan_segmenter.hpp
  include/slam_dynamic_filter/dynamic_tracker.hpp
  include/slam_dynamic_filter/scan_masker.hpp
  include/slam_dynamic_filter/session_health.hpp
  include/slam_dynamic_filter/metadata.hpp
  src/scan_validator.cpp
  src/motion_compensator.cpp
  src/scan_segmenter.cpp
  src/dynamic_tracker.cpp
  src/scan_masker.cpp
  src/session_health.cpp
  src/metadata.cpp
  src/slam_dynamic_filter_node.cpp
  src/safe_map_saver_node.cpp
  src/evidence_metrics.cpp
  src/offline_refiner.cpp
  srv/SaveCandidateMap.srv
  srv/ResetFilterSession.srv
  srv/SetFilterMode.srv
  config/r5_4500u.yaml
  config/simulation.yaml
  config/low_load.yaml
  launch/dynamic_filter.launch.py
  test/*.cpp
  test/*.py
  test/data/
```

`include/slam_dynamic_filter/types.hpp` 内容锁定：

- enum classes: `FilterMode`, `FilterState`, `ScanStatus`, `TrackStatus`, `SaveStatus`, `ReplayStatus`
- structs: `SessionId`, `FilterConfig`, `ScanFrame`, `Segment`, `Track`, `DynamicMask`, `HealthSnapshot`

Service contracts：

```text
srv/SaveCandidateMap.srv
string session_id
string request_id
string map_name
bool snapshot
---
bool accepted
string status_code
string message
string output_uri

srv/ResetFilterSession.srv
string expected_session_id
---
bool success
string new_session_id
string status_code
string message

srv/SetFilterMode.srv
string expected_session_id
string mode
---
bool success
string status_code
string message
```

现有包修改范围：

- 修改 `cod_bringup` launch、params、CMake、package：接入 `disabled|observe|enforce`、`/scan_raw`、`/scan_slam_filtered`、saver/session services、安装依赖。
- 替换 `auto_save_map` 行为，或保留 deprecated wrapper 但默认 disabled 且不 shell 保存。
- 修改 Gazebo launch/config/model/test world/docs：仅 smoke launch/QoS/state，不作为 MID-360 evidence。
- 增加 `scripts/record_mapping_session.sh` 和 `scripts/analyze_mapping_session.py`。
- 增加 docs runbook：record、observe、enforce、save denial、rollback、offline replay、R5 profiling。

Authoritative state：`DiagnosticArray` 仅观测。`safe_map_saver` 从共享 `SessionHealth` 和 typed requests 读取权威状态，不解析 free-form diagnostics。

ASCII dependency graph：

```text
                            +---------------------------+
                            | /livox/lidar raw cloud    |
                            +-------------+-------------+
                                          |
                                          v
                              cpp_lidar_filter
                                          |
                             /livox/lidar_filtered
                    +---------------------+---------------------+
                    |                                           |
                    v                                           v
         Nav2 STVL local/global costmaps              pointcloud_to_laserscan
              no-delay safety branch                         |
                    |                                      /scan_raw
                    |                                           |
                    |                         +-----------------+-----------------+
                    |                         | slam_dynamic_filter core library |
                    |                         | validate/compensate/segment/     |
                    |                         | track/mask/session/metadata      |
                    |                         +---------+---------------+--------+
                    |                                   |               |
                    v                                   v               v
              controller/cmd_vel                /scan_slam_filtered   diagnostics
                                                        |             markers
                                                        v
                                                  slam_toolbox
                                                        |
                                                        v
                                             nav2_map_server services
                                                        |
                                                        v
                                                safe_map_saver

Offline shared library branch:

  recorded bag + frozen metadata
      -> evidence_metrics CLI
      -> offline_refiner CLI
           -> same slam_dynamic_filter core library
           -> deterministic masks/tracks
           -> offline slam_toolbox run
           -> candidate map/report/hash
```

Concurrency contract：

- One bounded worker for scan/filter processing; no detached threads.
- Ingress uses SensorDataQoS and bounded queues with explicit overflow status.
- Session transition and save path use mutual exclusion; one active save maximum.
- `HealthSnapshot` is immutable once captured for a save/replay decision.
- Config update is atomic: validate full config, then swap; invalid config leaves prior config active.
- ROS adapters are thin; pure core receives explicit timestamps and inputs.

QoS contracts：

- Sensor ingress: SensorDataQoS, best effort acceptable only for raw high-rate sensor streams, bounded depth from config.
- Health/session/service: reliable, transient durability only where required for latched state, bounded request timeouts.
- Diagnostics/markers: non-authoritative observational topics; marker publication may skip expensive work if no subscribers.
- Saver services: reliable request/response with typed status and request ID idempotence.

Lifecycle behavior：

- Startup: `STARTING -> WARMING`; save denied.
- Warmup: enough valid scan/TF/odom/recorder evidence transitions to `HEALTHY`.
- Sustained invalid input, TF outage, high dynamic ratio, queue overflow, recorder failure or stale health transitions to `DEGRADED` or `FROZEN`; save denied.
- Node death or unrecoverable config/schema mismatch transitions to `ERROR`.
- Reset creates new session ID and clears session memory; stale requests with old session ID fail.

### Code Quality

Decisions：

- Pure deterministic core; ROS nodes only adapt topics/services/parameters.
- Same core library is used online and offline; no duplicated tracker logic.
- Explicit result enums; no bool-only errors.
- Config validation is atomic and all config failures produce typed result + diagnostic/log.
- Central metadata schema version is `1`.
- No custom target track message v1; use `DiagnosticArray`, JSON/CSV report and `MarkerArray` visualization only.
- Break modules into small methods with <=5 branches per method: validate geometry, validate values, interpolate motion, detect jump, segment, associate, confirm, mask, publish.
- Existing `package.xml` TODO license is unrelated debt; do not copy it into the new package.
- Existing `auto_save_map` shell path must be deleted or disabled by default and replaced by typed service behavior.

Module quality rules：

| Module | Rule |
|---|---|
| `scan_validator` | No state mutation; returns `ScanStatus` and normalized `ScanFrame`. |
| `motion_compensator` | Explicit exact/interpolated/missing/extrapolated TF outcomes; odom jump never hidden. |
| `scan_segmenter` | Deterministic split/merge order; stable segment IDs only within frame. |
| `dynamic_tracker` | Bounded candidates/tracks; confirmation requires persistence; ambiguous association cannot promote dynamic. |
| `scan_masker` | Dynamic -> `NaN`; static unchanged; real `inf` preserved; all unknown blocks save. |
| `session_health` | Single authority for save eligibility; immutable snapshots. |
| `metadata` | Schema v1 writer/reader; schema mismatch blocks final promotion. |
| ROS nodes | Thin adapters, no algorithm branching beyond parameter/service/topic glue. |

### Test Diagram

Existing tests are zero in repo for this feature; all below are planned TDD. No LLM eval.

```text
Evidence metrics [real bag, C1]
  raw cloud
    -> filtered cloud
      -> scan projection
        -> per-bin hit / persistence / scan-rate / coverage
        -> static-wall residual / LIO continuity-drift around dynamic targets
        -> compensated static consistency during straight/lateral/rotation motion
        -> whole-system profile
  Coverage: Unit(metrics parser), ROS integration/launch(recorder wiring),
            real-bag(required C1), Gazebo smoke(topic shape only), perf/chaos(R5)

Scan validation
  valid
  empty
  all-inf
  NaN
  invalid geometry
  timestamp reverse
  Coverage: Unit(all branches), ROS integration(fixture publisher), real-bag(observed stats)

Motion compensation
  exact TF
  interpolated TF
  missing TF
  extrapolated TF
  odom jump
  Coverage: Unit(tf/odom math), ROS integration(tf delay/missing), perf/chaos(TF outage)

Segmentation
  noise
  wall
  split/merge
  pitched scan
  Coverage: Unit(synthetic scans), Gazebo smoke(gross obstacle), real-bag(C1 pitched projection)

Tracking
  candidate
  confirmed
  stopped
  ambiguous crossing
  occlusion/reactivation
  over-capacity
  high dynamic ratio
  Coverage: Unit(state machine), ROS integration(observe diagnostics), real-bag(dynamic clips),
            perf/chaos(over-capacity/burst)

Mask
  dynamic NaN
  untouched static
  real inf
  all unknown
  Coverage: Unit(mask values), ROS integration(slam input topic), integration(NaN/inf SLAM behavior)

State/session
  startup
  warm
  healthy
  degraded
  frozen
  error
  reset
  restart
  session mismatch
  Coverage: Unit(state transitions), ROS integration(services), launch(node death)

Modes
  disabled
  observe
  enforce
  Coverage: launch tests, Gazebo smoke, rollback integration

Saver
  healthy
  stale
  duplicate
  wrong session
  path traversal/symlink
  disk low
  map service timeout/false
  Coverage: Unit(path/session), ROS integration(fake map service), perf/chaos(low disk)

Recorder
  complete
  process death
  low disk
  metadata missing
  Coverage: Unit(metadata), ROS integration(process supervisor), real-bag(completeness)

Offline
  stable order
  duplicate timestamps
  corrupt bag
  frozen schema mismatch
  repeated runs tolerance
  SLAM timeout
  Coverage: Unit(order/schema), CLI integration, real-bag replay, chaos(timeout/corrupt)

Integration
  STVL no-delay
  slam input delayed
  node death
  rollback
  Coverage: ROS launch, Gazebo smoke, real robot observe/enforce gates

Performance/chaos
  burst
  TF outage
  CPU pressure
  temp throttling
  2h
  Coverage: perf/chaos only; R5 profile required before acceptance
```

Count summary：约 45 个分支，全部映射到 Unit、ROS integration/launch、real-bag、Gazebo smoke、perf/chaos 中至少一种测试；除 C1 acceptance 外无未分配测试。C1 gate 未解决。

### Test Plan Artifact

测试计划外部 artifact 路径：`/home/wangtao/.gstack/projects/qing199822-ZZZL_nav2_real-world_application/wangtao-improve-eng-review-test-plan-20260719-203000.md`。该文件在本次单文件编辑之后创建；本次不创建，因为当前要求只编辑一个文件。

### Performance

保留 Phase 1 filter per-process 既有值：

- average filter load <=50% of one core。
- short peaks only。
- extra RSS <200 MB。
- p95 frame processing <30 ms。
- p95 SLAM delay within 0.8-1.3 s。
- Nav2 avoidance added latency <10 ms。
- bounded queues。
- no leaks/crashes over 2h。
- offline work capped to two cores。

新增 whole-machine acceptance。以下数字是 provisional gates，必须通过 R5 baseline calibration 固化；但 evidence report 必须给出 pass/fail，不允许只给图：

| Metric | Provisional gate |
|---|---|
| Total CPU sustained | <=75% whole machine sustained during mapping profile |
| Single core pinning | no single core pinned >95% for >2s except known controller burst |
| Controller loop | achieved >=45Hz p95 window for configured 50Hz controller |
| `cmd_vel` pipeline | p95 added latency <=10ms |
| Scan drop | <=1% over mapping session, with burst windows reported |
| TF latency/failure | TF p95 <=10ms and failure <0.1% |
| Thermal | no thermal throttling below 80% nominal sustained frequency over 2h |
| RSS/free memory | total RSS leaves >=1GB free |
| Bag write impact | no control jitter regression >5% while recording |

Load shedding order：

```text
1. Skip optional MarkerArray visualization.
2. Reduce diagnostics/report publish frequency, keep state snapshots.
3. Drop non-authoritative evidence detail, keep raw required recorder topics.
4. Reduce tracker candidate cap and enter DEGRADED if cap binds.
5. Freeze dynamic promotions; publish unmodified scan or observe-only by mode.
6. Deny save and require operator intervention.
7. Roll back enforce -> observe -> disabled if control or freshness safety degrades.
```

### Security and Error Acceptance

Direct acceptance criteria：

- No `bash -c` saver and no shell invocation for map save.
- Fixed canonical output root; all output paths must remain contained after symlink resolution.
- Path traversal and symlink containment tests are required.
- Bounded serialized bag/message sizes and bounded recorder duration/size policy.
- Topic allowlist for offline replay; unexpected topic/type fails closed.
- One active save maximum; duplicate request IDs are idempotent or return `SAVE_IN_PROGRESS`.
- Disk reserve checked before and during recorder/save.
- Typed request IDs and session IDs on save/reset/mode services.
- Trusted DDS boundary documented; ROS service callers are not treated as authenticated users.

Every named error from Phase 1 and Phase 3 has result enum, test, diagnostic/log and recovery. New result categories include config schema mismatch, QoS mismatch/no subscriber, DDS service duplicate, process restart mid-session, clock reset, duplicate timestamps and stale package install path.

### Deterministic Replay Contract

Exact contract：

- Record raw+filtered clouds, actual MID-360 built-in IMU topic, `scan_raw`, mask, diagnostics, `tf`, `tf_static`, odom.
- Record frozen launch/YAML/driver config/git SHA/build metadata/session ID, including the actual MID-360 driver config used for the bag.
- Stable ordering key is `(source timestamp, topic priority, bag sequence)`.
- Duplicates are preserved by bag sequence.
- Replay uses `use_sim_time` + fixed-rate `/clock`.
- Replay uses single-thread core library.
- Replay makes no wall-clock decisions.
- Deterministic seeds are set where relevant.
- `slam_toolbox` mode/config is explicit in metadata.
- Async SLAM output is compared by tolerance, not byte equality.
- The same fixture runs 3 times.
- Masks/tracks must be exactly equal across the 3 runs.
- Map aligned-grid occupancy differs <=0.1% cells unless calibration tightens/loosens this in a versioned report.
- Pose graph/map metadata hashes are recorded.
- Schema mismatch blocks final promotion.

### Failure Modes Registry

Phase 1 registry is reused. Added Phase 3 failures：

| Failure | Result enum | Test | Diagnostic/log | Recovery |
|---|---|---|---|---|
| Config schema mismatch | `ReplayStatus::SCHEMA_MISMATCH` or `FilterState::ERROR` | unit + offline fixture | schema expected/actual logged | block final promotion, keep prior online config |
| QoS mismatch/no subscriber | `ScanStatus::QOS_OR_SUBSCRIBER_MISMATCH` | ROS integration | topic, QoS profile, subscriber count | observe warning, skip optional output, deny enforce if SLAM not subscribed |
| DDS service duplicate | `SaveStatus::DUPLICATE_REQUEST` | service integration | request_id/session_id | idempotent response or `SAVE_IN_PROGRESS` |
| Process restart mid-session | `FilterState::RESTARTED_SESSION` | launch kill/restart | old/new session IDs | create new session, reject stale requests |
| Clock reset | `ReplayStatus::CLOCK_RESET` / `ScanStatus::TIMESTAMP_REVERSED` | replay + online fixture | prior/current time | abort replay or reset session online |
| Duplicate timestamps | `ReplayStatus::DUPLICATE_TIMESTAMP` | replay order test | sequence and topic | preserve by bag sequence |
| Stale package install path | `ReplayStatus::STALE_INSTALL_METADATA` | metadata fixture | build/install/git mismatch | block final promotion and request rebuild |

After these constraints there are no critical silent gaps. C1 remains unresolved feasibility, not a silent runtime failure.

### Deployment and Parallelization

Dependency table：

| Work | Depends on | Can parallelize | Conflict flags |
|---|---|---|---|
| Foundation: evidence/session types + package scaffolding | none | no, starts first | new package names, generated interfaces |
| Lane A core algorithms unit TDD | Foundation | with Lane B/C | shared `types.hpp`, config schema |
| Lane B saver/session typed service TDD | Foundation | with Lane A/C | `cod_bringup` launch/params, map saver behavior |
| Lane C recorder/evidence metrics TDD | Foundation | with Lane A/B | scripts, docs, recorder metadata |
| Merge launch/observe integration | Lanes A/B/C minimal interfaces | after lane merge | `cod_bringup` launch/params and package shared |
| Enforce rollout | C1 pass + observe metrics + Nav2 safety | no | SLAM remap, safety gates |
| Phase B calibration | observe/enforce candidates | after C1 data | thresholds and performance gates |
| Phase C offline | stable metadata/core | after schema/core freeze | offline CLI and SLAM configs |

Because project CLAUDE requires worktree later, execution plan creates a worktree only at implementation stage, not during this documentation review.

Sequential plan：

```text
Foundation
  evidence/session types + package scaffolding

Parallel lanes
  Lane A: core algorithms unit TDD
  Lane B: saver/session typed service TDD
  Lane C: recorder/evidence metrics TDD

Merge
  launch/observe integration

Gate
  enforce only after C1 pass

Later
  Phase B calibration
  Phase C offline
```

Rollout：

```text
disabled -> observe -> enforce
rollback: enforce -> observe -> disabled
```

Rollback preserves bags, candidate maps, reports and session metadata; final promotion remains blocked when state/session/schema/evidence is invalid.

### What already exists

| Asset | Existing role | Phase 3 decision |
|---|---|---|
| `cpp_lidar_filter` | Crop/optional voxel on Livox cloud | Reuse before SLAM filter and STVL branch |
| `pointcloud_to_laserscan` | Cloud-to-scan projection | Reuse but measure; create `/scan_raw` and `/scan_slam_filtered` wiring |
| `small_point_lio` | `/Odometry`, `/cloud_registered`, odom TF | Same-source candidate odom only until C1 evidence; if evidence fails, remain observe and never enforce |
| Nav2 STVL | No-delay obstacle marking/clearing from filtered cloud | Keep direct branch; add freshness/collision safety |
| `slam_toolbox` | Online SLAM | Reuse with explicit input remap and replay config |
| `nav2_map_server` | Map save service | Use typed service client from `safe_map_saver` |
| `auto_save_map` | Unconditional shell map save | Delete behavior or keep disabled deprecated wrapper |
| `scripts/record_bag.sh` | Basic Livox bag recorder pattern | Reuse style; create mapping-specific recorder |
| Gazebo launch/world/docs | Sim smoke environment | Keep smoke-only and label fidelity limits |

### NOT in scope

- No semantic camera/person/box classifier.
- No cloud processing or external dashboard.
- No external package publishing.
- No cross-session dynamic memory or permanent forbidden-object map.
- No replacement of `slam_toolbox`, Nav2, STVL or `small_point_lio` unless C1 evidence invalidates the premise.
- No custom target track ROS message in v1.
- No final-map auto-promotion.
- No LLM eval.
- No GSTACK REVIEW REPORT in this edit.
- No independent wheel/electrical IMU requirement in current acceptance; it is a future NOT-in-scope hardware option only.

### Engineering Implementation Tasks

| Priority | Effort | Task | Exact files |
|---|---:|---|---|
| P1 | M | Evidence recorder and metrics | `scripts/record_mapping_session.sh`, `scripts/analyze_mapping_session.py`, `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/src/evidence_metrics.cpp`, `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/include/slam_dynamic_filter/metadata.hpp`, `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/src/metadata.cpp` |
| P1 | M | Package/types/state scaffolding | `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/package.xml`, `CMakeLists.txt`, `include/slam_dynamic_filter/types.hpp`, `include/slam_dynamic_filter/session_health.hpp`, `src/session_health.cpp`, `srv/SaveCandidateMap.srv`, `srv/ResetFilterSession.srv`, `srv/SetFilterMode.srv` |
| P1 | M | Validator/compensator | `include/slam_dynamic_filter/scan_validator.hpp`, `src/scan_validator.cpp`, `include/slam_dynamic_filter/motion_compensator.hpp`, `src/motion_compensator.cpp`, `test/test_scan_validator.cpp`, `test/test_motion_compensator.cpp` |
| P1 | L | Segmenter/tracker/mask | `include/slam_dynamic_filter/scan_segmenter.hpp`, `src/scan_segmenter.cpp`, `include/slam_dynamic_filter/dynamic_tracker.hpp`, `src/dynamic_tracker.cpp`, `include/slam_dynamic_filter/scan_masker.hpp`, `src/scan_masker.cpp`, `test/test_scan_segmenter.cpp`, `test/test_dynamic_tracker.cpp`, `test/test_scan_masker.cpp` |
| P1 | M | Filter node observe/enforce | `src/slam_dynamic_filter_node.cpp`, `launch/dynamic_filter.launch.py`, `config/r5_4500u.yaml`, `config/simulation.yaml`, `config/low_load.yaml`, `test/test_dynamic_filter_launch.py` |
| P1 | M | Safe saver typed service | `src/safe_map_saver_node.cpp`, `srv/SaveCandidateMap.srv`, `srv/ResetFilterSession.srv`, `srv/SetFilterMode.srv`, `test/test_safe_map_saver.cpp`, `test/test_safe_map_saver_launch.py` |
| P1 | M | Nav2 freshness/collision safety | `cod_-rm2026_-navigation-master/src/cod_bringup/params/multiplenav2_params.yaml`, `cod_-rm2026_-navigation-master/src/cod_bringup/launch/multiplenav_launch.py`, `cod_-rm2026_-navigation-master/src/cod_bringup/CMakeLists.txt`, `cod_-rm2026_-navigation-master/src/cod_bringup/package.xml` |
| P1 | M | Launch modes and disabled autosave | `cod_-rm2026_-navigation-master/src/cod_bringup/launch/multiplenav_launch.py`, `cod_-rm2026_-navigation-master/src/cod_bringup/launch/auto_save_map.launch.py`, `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/launch/dynamic_filter.launch.py` |
| P1 | L | Tests and fixtures | `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/*.cpp`, `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/*.py`, `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/test/data/` |
| P1 | M | R5 profile | `scripts/analyze_mapping_session.py`, `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/config/r5_4500u.yaml`, docs runbook |
| P2 | L | Offline contract/refiner | `src/offline_refiner.cpp`, `include/slam_dynamic_filter/metadata.hpp`, `src/metadata.cpp`, `test/test_offline_refiner.cpp`, `test/test_replay_determinism.py` |
| P2 | M | Docs | docs runbook, Gazebo docs, quick-start mapping diagram updates |

### ENG DUAL VOICES — CONSENSUS TABLE

| Dimension | Claude | Codex findings | Consensus |
|---|---|---|---|
| 1 Architecture | N/A | New package, pure core, thin ROS adapters, STVL no-delay branch, typed services, offline shared library; C1 evidence before tracker enforce | N/A |
| 2 Error/rescue | N/A | Every Phase 1/3 error mapped to enum, test, diagnostic/log and recovery; no bool-only errors | N/A |
| 3 Security | N/A | No shell saver, canonical root, symlink containment, bounded bag/message sizes, topic allowlist, one active save, disk reserve, typed IDs, trusted DDS boundary | N/A |
| 4 Data flow | N/A | Recorder must include raw+filtered clouds, actual MID-360 built-in IMU topic, MID-360 driver config, scan_raw, mask, diagnostics, tf/tf_static, odom and frozen metadata | N/A |
| 5 Performance | N/A | Retain filter gates and add whole-machine provisional gates requiring R5 baseline with explicit pass/fail | N/A |
| 6 Rollout | N/A | Workstreams staged; disabled -> observe -> enforce; Gazebo smoke-only; enforce blocked until C1 pass | N/A |

### Engineering Completion Summary

| Item | Result |
|---|---|
| Scope | Preserved but staged: minimum safe hardening first, online core second, offline refinement third |
| Architecture findings | 5: evidence gate, typed services, pure core/thin adapters, STVL direct branch, offline shared library |
| Code quality findings | 4: deterministic core, explicit enums, atomic config validation, metadata schema v1 |
| Test diagram | ~45 branches, 0 unassigned except C1 acceptance; existing feature tests are zero and all are planned TDD |
| Performance | 2 classes: retained filter per-process gates and new whole-machine provisional gates |
| Critical gaps | 0 after proposed constraints; C1 remains unresolved feasibility |
| Outside voice | Codex-only; Claude N/A due `deepseek-v4-flash` environment config |
| Lanes | Foundation, Lane A core, Lane B saver/session, Lane C recorder/evidence, merge observe, enforce after C1, Phase B calibration, Phase C offline |
| Lake score complete options | A evidence/safety first is lowest risk; B full approved architecture remains target after C1; C reviewed-map workflow remains fallback if C1 fails |

Phase 3 complete. Passing to DX.

## Phase 3.5 — DX Review

目标读者是比赛机器人 ROS2 部署/标定工程师。主要产品形态是 CLI、ROS package 和文档；模式是 DX POLISH。本阶段不实现代码，只把可交付的开发者体验契约补齐到可以进入 Phase 4 final gate。

### Developer Persona Card

| Field | Detail |
|---|---|
| Persona | 比赛机器人 ROS2 deployment/calibration engineer |
| Hardware | R5-4500U 车载机，Livox MID-360 pointcloud + built-in IMU only |
| Missing hardware | 无电控 IMU、无独立 wheel odom、无 Realsense |
| Context | 实验室调图、赛前准备、场地变化后校准 |
| Tolerance | 首次证据可以等 15 min，但安装后回访状态/健康检查必须 <5 min |
| Expectation | copy-paste commands、fail-closed gates、可读报告、稳定 exit codes |
| Refusal point | 不接受手写 ROS service YAML；不接受文档要求不存在的传感器 |
| Safety posture | 正常比赛导航默认使用 reviewed static maps；动态过滤进入 enforce 必须有 C1 gate |

### First-Person Empathy Narrative

我是负责把 R5-4500U 机器人带进实验室和赛场的 ROS2 部署/标定工程师。我读到 README 的一句话时，理解这个仓库是面向真实机器人导航应用的工程集合，但我马上会去找最短路径：先确认传感器、再录一段证据、再判断能不能进入动态过滤。现在的体验会让我犹豫。QUICK_START 里有 raw `map_saver_cli`，看起来可以直接保存地图，但它没有告诉我这个地图是否来自 reviewed static map、candidate map，还是某次未验证 session。docs/QUICK_START 又让我直接跑 `multiplenav` 并检查 `/scan`，这对普通导航有效，却没有把 `/scan_raw`、过滤后 scan、same-source `/Odometry`、MID-360 built-in IMU 和 C1 gate 串成一条可复制链路。HOST_SETUP 估计 60-90 min，还安装 Realsense；我的车上没有这类硬件，这让我怀疑文档是否理解当前机器人边界。我可以接受第一次跑 evidence 需要 2 min bag 加分析，总计 15 min 内给出结论；但我不能接受每次回到实验室都重新读一堆 ROS 命令。理想路径是：build 后复制 `sdfctl doctor`，看到硬件边界和 topic preflight；录 120 秒 static wall rotation；`analyze_session` 给出 C1 pass 或 blocked；以后用 `sdfctl health` 在 5 min 内知道 mode、session、gate、save eligibility 和下一条修复命令。

### Competitive / Reference Benchmark

本仓库此前没有 prior DX review。本表把外部 ROS 常见流程转成 robotics internal tool benchmark，不作为市场竞品声明。

| Reference flow | Time to status | Friction | Operator burden | Adapted target |
|---|---:|---|---|---|
| 标准 `ros2 topic/service` 手工流程 | >15 min | High | 手写 service YAML、topic 名靠记忆、结果靠肉眼判断 | 只保留为底层 debug，不作为主入口 |
| `ros2 doctor` / preflight style | 2-5 min | Medium | 机器能列环境状态，但不能知道本功能 gate | `sdfctl doctor` 和 `sdfctl health` 输出 feature-specific decisions |
| Target `sdfctl health` | <1 min after build/source | Low | 一条命令显示 mode/session/gates/save eligibility | 回访开发者 <5 min 完成状态闭环 |
| Target first evidence sequence | <=15 min | Low-Medium | 必须录 2 min bag 和实验场景，所以不能压到 1 min | build 后首次 C1 evidence pass/blocked 可机械执行 |

### Magical Moment

魔法时刻不是 GUI，也不是 hosted playground，而是 terminal 里出现机械可执行的结论。底层 `analyze_session` 输出 C1 pass 或 blocked report，CLI 用同一结论作为 operator contract：

```bash
sdfctl analyze --session 20260719-142233-static_wall_rotation
```

期望 final line：

```text
C1_EVIDENCE_PASS allow_enforce=true gate=/home/$USER/cod_mapping_sessions/20260719-142233-static_wall_rotation/evidence/c1_gate.json
```

或 fail-closed：

```text
C1_EVIDENCE_BLOCKED code=C1_ODOM_INCONSISTENT problem="same-source odom drift exceeded threshold" measured="0.42m > 0.20m" fix="rerun static_wall_rotation with stable MID-360 mount and check /Odometry timestamp source" retry="sdfctl record --duration 120 --scenario static_wall_rotation && sdfctl analyze --session <new_id>" docs="docs/MAPPING_DX_RUNBOOK.md#c1-odom-inconsistent"
```

回访时：

```bash
sdfctl health
```

必须显示当前 mode、active session、C1 gate、save eligibility、candidate state 和 exact fixes。交付方式是 copy-paste terminal commands；没有 hosted playground。

### Hard Defaults

这些默认值必须硬编码到 CLI/launch/config 契约，并在 `sdfctl doctor` 和 `sdfctl health --json` 中可见：

```text
mode=observe
final_promotion=false
candidate_snapshot=false
output_root=$HOME/cod_mapping_sessions
max_record_duration_sec=300
min_disk_free_gb=20
require_c1_for_enforce=true
normal competition navigation defaults to reviewed static maps
```

### CLI Contract

可执行文件名固定为 `sdfctl`，全称是 SLAM Dynamic Filter control。CLI 封装 typed ROS services；operators never type YAML。mode service 使用 constants/int enum，不使用 free string。Exit codes 稳定并写入文档。

| Command | Purpose | Contract |
|---|---|---|
| `sdfctl doctor` | 环境和 topic preflight | 检查硬件边界、ROS graph、topic、依赖、磁盘、R5 profile |
| `sdfctl record --duration 120 --scenario <name>` | 录制 evidence session | duration 不得超过 `max_record_duration_sec=300` |
| `sdfctl analyze --session <id>` | 生成 C1 evidence | 输出 pass 或 blocked final line，并写 `c1_gate.json` |
| `sdfctl health [--json]` | 当前功能状态 | 显示 mode/session/gates/save eligibility/fixes |
| `sdfctl mode observe\|enforce\|disabled` | 切换模式 | 底层调用 enum typed service；enforce 必须匹配 gate |
| `sdfctl save-candidate --name <safe-name>` | 保存候选地图 | 写 session artifact layout，不直接写 `cod_bringup/maps` |
| `sdfctl offline-refine --session <id> --max-cores 2` | 离线 refine | 限制 R5 CPU，生成 diff/report/candidate |
| `sdfctl promote --session <id> --candidate <name>` | 人工 promotion | 需要 human-reviewed report 和 typed confirmation |
| `sdfctl rollback observe\|disabled` | 运行时回退 | 不假设 systemd；切回 observe/disabled 并要求 relaunch arg 对齐 |
| `sdfctl verify-rollback` | 验证回退 | 检查 active_map manifest、mode、topics 和 saver disabled |
| `sdfctl metadata migrate --from 1 --to 2 --dry-run --session <id>` | schema dry-run | v1 当前无迁移；v2 出现后可 dry-run |

### First Evidence Sequence

目标：built workspace 后 15 min 内获得 first evidence；回访开发者运行 `sdfctl health` 必须 <5 min。

```bash
colcon build --symlink-install --packages-up-to slam_dynamic_filter
source install/setup.bash
sdfctl doctor
```

期望输出必须包含硬件边界和 topic preflight：

```text
SDFCTL_DOCTOR_OK
hardware.r5_4500u=true
hardware.mid360=true
same_source_odom=true
electrical_imu=false
topic./livox/lidar=ok
topic./livox/imu=ok
topic./Odometry=ok
topic./tf=ok
topic./tf_static=ok
fix_if_blocked="docs/MAPPING_DX_RUNBOOK.md#doctor"
```

录制 evidence：

```bash
sdfctl record --duration 120 --scenario static_wall_rotation
```

期望输出：

```text
SDFCTL_RECORD_OK session=20260719-142233-static_wall_rotation path=/home/$USER/cod_mapping_sessions/20260719-142233-static_wall_rotation
```

分析 evidence：

```bash
sdfctl analyze --session 20260719-142233-static_wall_rotation
```

final output 只能是：

```text
C1_EVIDENCE_PASS allow_enforce=true
```

或：

```text
C1_EVIDENCE_BLOCKED code=C1_SCAN_UNSTABLE fix="sdfctl record --duration 120 --scenario static_wall_rotation && sdfctl analyze --session <new_id>"
```

机械 gate file 固定为：

```text
~/cod_mapping_sessions/<id>/evidence/c1_gate.json
```

字段必须包含 `scan_stability`、`same_source_odom_consistency`、`r5_profile`、`schema_version`、`config_hash`、`git_sha`、`allow_enforce`。没有 matching gate 时 enforce impossible。

### Evidence Recorder Fail-Fast Topic List

Recorder 启动时必须 fail fast 检查并记录：

| Required now | Purpose |
|---|---|
| `/livox/lidar` | MID-360 raw pointcloud |
| `/livox/lidar_filtered` | filter output evidence |
| `/livox/imu` | actual MID-360 built-in IMU |
| `/scan_raw` | raw projection before dynamic mask |
| `/Odometry` | `small_point_lio` same-source odom |
| `/tf` | transform stream |
| `/tf_static` | static transforms |

Later diagnostics/mask topics 也进入 allowlist，但不能替代 required topics。Session metadata 必须保存 params、driver、git、build、session。必须显式写入 `same_source_odom=true`。不得请求 wheel odom 或 electrical IMU。

### Actionable Error Registry

每个输出必须有 code、problem、measured cause vs threshold、exact command 或 file to fix、retry command、docs anchor。Diagnostics keys 不是 decisions；typed health 和 `sdfctl` 计算 decisions。

| Code | Problem | Cause / threshold | Fix | Retry | Docs anchor |
|---|---|---|---|---|---|
| `C1_SCAN_UNSTABLE` | C1 scan 不稳定 | `scan_stability` below threshold, measured in report | 固定 MID-360，清空动态障碍，重录 static wall rotation | `sdfctl record --duration 120 --scenario static_wall_rotation && sdfctl analyze --session <new_id>` | `docs/MAPPING_DX_RUNBOOK.md#c1-scan-unstable` |
| `C1_ODOM_INCONSISTENT` | same-source odom 不一致 | `/Odometry` drift or timestamp mismatch exceeds threshold | 检查 Livox driver time sync 和 `small_point_lio` config | `sdfctl doctor && sdfctl analyze --session <id>` | `docs/MAPPING_DX_RUNBOOK.md#c1-odom-inconsistent` |
| `FILTER_TF_STALE` | filter TF 过期 | latest TF age above threshold | 检查 `/tf` publisher 和 launch ordering | `sdfctl health` | `docs/MAPPING_DX_RUNBOOK.md#filter-tf-stale` |
| `FILTER_QUEUE_OVERFLOW` | filter queue 溢出 | dropped messages exceed threshold | 使用 R5 profile，降低 debug logs，检查 CPU thermal | `sdfctl record --duration 120 --scenario static_wall_rotation` | `docs/MAPPING_DX_RUNBOOK.md#filter-queue-overflow` |
| `MODE_GATE_MISSING` | 无 gate 切 enforce | `c1_gate.json` missing or config/git mismatch | 先执行 record/analyze，或切回 observe | `sdfctl mode observe && sdfctl health` | `docs/MAPPING_DX_RUNBOOK.md#mode-gate-missing` |
| `SAVE_SESSION_MISMATCH` | 保存 session 不匹配 | active session != candidate session | 使用当前 session 的 candidate name 或重新 record | `sdfctl save-candidate --name <safe-name>` | `docs/MAPPING_DX_RUNBOOK.md#save-session-mismatch` |
| `SAVE_LOW_DISK` | 磁盘不足 | free disk below `min_disk_free_gb=20` | 清理 `~/cod_mapping_sessions` 旧 bag 或换盘 | `sdfctl health` | `docs/MAPPING_DX_RUNBOOK.md#save-low-disk` |
| `SAVE_MAP_SERVICE_TIMEOUT` | map save service 超时 | typed save service timeout exceeded | 检查 map_server lifecycle 和 `/map` publisher | `sdfctl save-candidate --name <safe-name>` | `docs/MAPPING_DX_RUNBOOK.md#save-map-service-timeout` |
| `BAG_TOPIC_MISSING` | bag 缺 required topic | missing topic from fail-fast list | 重录；不要用旧 Livox-only bag 当 C1 evidence | `sdfctl record --duration 120 --scenario static_wall_rotation` | `docs/MAPPING_DX_RUNBOOK.md#bag-topic-missing` |
| `BAG_METADATA_MISMATCH` | bag metadata 不匹配 | schema/config/git/session mismatch | 使用同一 build/source 重新 analyze 或 migrate dry-run | `sdfctl metadata migrate --from 1 --to 2 --dry-run --session <id>` | `docs/MAPPING_DX_RUNBOOK.md#bag-metadata-mismatch` |
| `REPLAY_NONDETERMINISTIC` | replay 非确定 | repeated replay diff above threshold | 检查 timestamp order、duplicate sequence、`use_sim_time` | `sdfctl offline-refine --session <id> --max-cores 2` | `docs/MAPPING_DX_RUNBOOK.md#replay-nondeterministic` |
| `PROMOTION_REVIEW_REQUIRED` | promotion 缺人工 review | human-reviewed report absent or typed confirmation missing | 打开 report，完成 review checklist，再输入确认 | `sdfctl promote --session <id> --candidate <name>` | `docs/MAPPING_DX_RUNBOOK.md#promotion-review-required` |

### Artifact Layout

所有动态 SLAM 过滤证据写入：

```text
~/cod_mapping_sessions/<session_id>/
  evidence/c1_gate.json
  bag/
  config/
  logs/
  reports/
  maps/<candidate>/
    map.yaml
    map.pgm
    metadata.json
    health.json
    save_request.json
  refined/
  active_map.json
```

`active_map.json` 必须包含 current、previous、candidate。Candidate never auto promotes。系统 never write directly to `cod_bringup/maps`。现有 raw `map_saver_cli` docs 必须删除或标记 unsafe，并改为 `sdfctl save-candidate` 与 manual promote 路径。

### Offline Output and Rollback

`sdfctl offline-refine --session <id> --max-cores 2` 必须输出：

```text
offline_report.json
diff_report.json
maps/<candidate>/{map.yaml,map.pgm,metadata.json,health.json,save_request.json}
PROMOTION_ALLOWED false|true
```

Rollback 不假设项目存在 systemd service。操作命令固定为：

```bash
sdfctl rollback observe
sdfctl verify-rollback
```

如需要 disabled：

```bash
sdfctl rollback disabled
sdfctl verify-rollback
```

Launch 层用 relaunch arg 与 `active_map.json` manifest 对齐；verify 检查 mode、active map current/previous/candidate、save eligibility 和 stale candidate。

### Upgrade Contract

当前 metadata schema 是 v1，没有 migration needed。CLI contract 仍需预留：

```bash
sdfctl metadata migrate --from 1 --to 2 --dry-run --session <id>
```

v2 出现后必须满足：backward reader supports one prior schema、dry-run migration、migration tests、release checklist 包含 old session refineability。旧 session 不能因为小版本升级而失去 offline refine 能力。

### Dev Environment

必须提供 pinned `dependencies.repos` 或 docs manifest，包含 exact Livox SDK/driver commit、rosdep command、package-focused build/test、R5 no-GPU profile。此 feature 不要求 Realsense。

最低命令形态：

```bash
vcs import src < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to slam_dynamic_filter
colcon test --packages-select slam_dynamic_filter
```

CI 使用 synthetic fixture 和 mini bag，覆盖 build、unit、launch、fake saver、schema、replay。真实 bags 因 privacy/size external，不进入仓库。

### Runbook and Docs Links

新增单一 runbook：

```text
docs/MAPPING_DX_RUNBOOK.md
```

必须从 `README`、`QUICK_START.md`、`docs/QUICK_START.md`、`docs/livox-debug-guide.md` 链接。顺序固定：

```text
build -> doctor -> record evidence -> analyze -> observe -> diagnose -> enforce decision -> candidate save -> offline refine -> manual promote -> rollback -> upgrade
```

同步更新 raw `/scan` 和 `map_saver_cli` 命令：`/scan` 文档要区分 `/scan_raw` 与过滤后 scan；raw `map_saver_cli` 必须标记 unsafe 或移除，改指向 `sdfctl save-candidate`。

### Developer Journey

| Stage | Current friction | After-plan status |
|---|---|---|
| discover | README 一句话能说明项目方向，但不能说明动态过滤证据路径 | README 链到 Mapping DX runbook |
| evaluate | 不清楚 MID-360-only 是否足够，可能误以为要 Realsense/电控 IMU | `sdfctl doctor` 明确 hardware boundary |
| install | HOST_SETUP 60-90 min 且安装 absent Realsense | feature docs 使用 pinned manifest、rosdep、package-focused build |
| hello | `multiplenav` + `/scan` 检查不能产生 C1 结论 | 15 min 内 record/analyze 给 pass/blocked |
| integrate | 手工 ROS service YAML 容易出错 | `sdfctl` 封装 typed services |
| debug | diagnostics 有键值但 operator 不知道决策 | `sdfctl health` 计算 decisions 和 exact fixes |
| upgrade | schema 变化路径未定义 | v1/v2 migration contract 和 tests |
| scale/mapping session | bag/map/report 分散，候选地图容易覆盖 | 固定 session artifact layout |
| migrate/rollback | 可能假设 systemd 或直接覆盖 map | `active_map.json` + `sdfctl rollback observe|disabled` |

第一次开发者困惑报告：

| Timestamp | Current experience | Desired success/block |
|---:|---|---|
| 0:00 | 读 README，知道是导航工程，但不知道动态过滤入口 | README 链接 `docs/MAPPING_DX_RUNBOOK.md` |
| 2:00 | 打开 QUICK_START，看到 raw map save，担心覆盖正式地图 | runbook 明确 candidate never auto promotes |
| 4:00 | docs/QUICK_START 要跑 `multiplenav` 看 `/scan`，不知道 C1 要哪些 topic | `sdfctl doctor` 列 `/livox/lidar`、`/livox/imu`、`/Odometry`、TF |
| 6:00 | HOST_SETUP 安装 Realsense，硬件不存在 | doctor 输出 `electrical_imu=false` 且不要求 Realsense |
| 8:00 | 不知道 same-source odom 是否合格 | doctor 输出 `same_source_odom=true`，analyze 计算 consistency |
| 10:00 | 可能手写 ROS service YAML 切 mode | `sdfctl mode observe|enforce|disabled` |
| 12:00 | 保存地图前不知道 gate 是否允许 | `sdfctl health` 显示 save eligibility |
| 15:00 | 当前只能靠人工猜测成功/失败 | 输出 `C1_EVIDENCE_PASS allow_enforce=true` 或 blocked code/fix/retry |

### Eight DX Passes

| Dimension | Initial | After plan | Evidence |
|---|---:|---:|---|
| Getting started | 2 | 8 | 现在入口分散，README/QUICK_START/HOST_SETUP 之间存在硬件和命令歧义；计划通过 runbook、doctor、first evidence sequence 将 TTHW 从 undefined/>15 收敛到 health<5 和 first evidence<=15。 |
| API/CLI | 4 | 9 | 现在有 ROS tools 和脚本模式，但 operator 仍可能手写 YAML；计划用 `sdfctl` 包装 typed ROS services、稳定 exit codes 和 enum mode。 |
| Errors | 3 | 9 | 现在 diagnostics 能观察但不能替 operator 决策；计划所有输出包含 code/problem/measured threshold/fix/retry/docs anchor，由 typed health 计算 decisions。 |
| Docs | 3 | 8 | 现在 raw `/scan`、raw `map_saver_cli`、HOST_SETUP Realsense 和 direct `multiplenav` 容易混淆；计划单一 Mapping DX runbook 并从四处入口链接。 |
| Upgrade | 2 | 8 | 现在 schema v1 已规划但 migration path 不完整；计划 backward reader、dry-run migration、migration tests 和 release checklist。 |
| Dev env | 4 | 8 | 现在能靠现有 setup 起步，但依赖 pinning、Livox commit、R5 no-GPU profile 不够明确；计划 dependencies manifest、rosdep、package-focused build/test。 |
| Community | 3 | 6 | 内部 repo 不需要公开社区体系；只补 issue/ownership 信息、runbook owner、常见 blocked codes，不建设 public package release 或外部论坛。 |
| Measurement | 5 | 9 | Phase 3 已重视 evidence；DX 计划补 TTHW script、stable exit codes、evidence reports、post-implementation devex boomerang 来量化体验。 |

### DX Outside Voices

Claude subagent unavailable due `deepseek-v4-flash` environment config；Codex ran。本节不作 dual-model claim。

Codex 13 findings：

| # | Finding | Incorporated response |
|---:|---|---|
| 1 | 当前 README/QUICK_START 不能给动态过滤 first evidence 路径 | 增加 Mapping DX runbook 和 first evidence sequence |
| 2 | HOST_SETUP 安装 Realsense 与当前硬件不符 | feature docs 明确 Do not require Realsense |
| 3 | raw `map_saver_cli` 会误导 operator 直接保存不安全地图 | 删除或标记 unsafe，改用 candidate save |
| 4 | `/scan` 检查没有区分 raw 和 filtered | docs 更新 `/scan_raw` 与过滤后 scan |
| 5 | 手写 ROS service YAML 不适合比赛部署 | `sdfctl` 包装 typed ROS services |
| 6 | diagnostics keys 容易被误当决策 | `sdfctl health` 计算 typed decisions |
| 7 | same-source odom 边界需要显式写入 | doctor 和 c1 gate 均输出 `same_source_odom=true` |
| 8 | 无 electrical IMU/wheel odom 时不能阻塞 C1 | evidence 不请求这些 topic |
| 9 | session artifact 分散会破坏可复现 | 固定 `~/cod_mapping_sessions/<session_id>/` |
| 10 | enforce gate 需要机器文件而非人读日志 | `evidence/c1_gate.json` 成为唯一入口 |
| 11 | offline refine 和 promotion 边界不清 | 输出 reports、candidate、`PROMOTION_ALLOWED`，promotion 需人工确认 |
| 12 | rollback 不应假设 systemd | 使用 `sdfctl rollback observe|disabled` 和 verify |
| 13 | DX 成功需要可量化 | TTHW、exit codes、evidence reports、devex boomerang |

6-dimension consensus：

| Dimension | Claude | Codex concern | Consensus |
|---|---|---|---|
| Getting started | N/A | first evidence path fragmented | N/A |
| CLI/API | N/A | ROS service YAML too fragile | N/A |
| Error handling | N/A | diagnostics not decisions | N/A |
| Documentation | N/A | raw map saver and Realsense confuse hardware boundary | N/A |
| Safety/rollout | N/A | enforce must require matching C1 gate | N/A |
| Measurement | N/A | TTHW and exit-code contract missing | N/A |

### DX Scorecard

| Area | Initial | Planned | Notes |
|---|---:|---:|---|
| Overall DX | 3.25/10 | 8.4/10 | 内部比赛机器人工具，不追求 public polish |
| TTHW health | undefined/>15 min | <5 min returning developer | `sdfctl health` |
| TTHW first evidence | undefined/>15 min | <=15 min | includes required 2 min bag/scenes |
| Safety clarity | 4/10 | 9/10 | fail-closed defaults and C1 gate |
| Artifact determinism | 5/10 | 9/10 | fixed layout and manifest |
| Operator recovery | 3/10 | 9/10 | actionable error registry |

What exists now：QUICK_START docs、HOST setup、`record_bag` patterns、ROS diagnostics、RViz。

NOT in scope：hosted playground、cloud telemetry、GUI、semantic camera、independent electrical IMU/wheel odom、public package release。

No GSTACK REVIEW REPORT yet.

### DX Implementation Checklist

| Priority | Task | Exact paths | Commands/tests |
|---|---|---|---|
| P1 | `sdfctl` executable wrapper | `cod_-rm2026_-navigation-master/src/slam_dynamic_filter/scripts/sdfctl`, package install rules | `sdfctl doctor`, `sdfctl health --json`, CLI exit-code tests |
| P1 | doctor/recorder/analyzer | `scripts/sdfctl_doctor.py`, `scripts/sdfctl_record.py`, `scripts/sdfctl_analyze.py`, `scripts/analyze_mapping_session.py` | `sdfctl record --duration 120 --scenario static_wall_rotation`; fake topic launch tests |
| P1 | runbook/docs links | `docs/MAPPING_DX_RUNBOOK.md`, `README.md`, `QUICK_START.md`, `docs/QUICK_START.md`, `docs/livox-debug-guide.md` | docs link check; raw map_saver unsafe check |
| P1 | errors registry | `include/slam_dynamic_filter/error_codes.hpp`, `src/error_codes.cpp`, `docs/MAPPING_DX_RUNBOOK.md` | unit tests for every code and docs anchor |
| P1 | artifact manager/active map | `include/slam_dynamic_filter/artifact_manager.hpp`, `src/artifact_manager.cpp`, `test/test_artifact_manager.cpp` | path containment, low disk, active_map manifest tests |
| P1 | schema migration contract | `include/slam_dynamic_filter/metadata.hpp`, `src/metadata.cpp`, `scripts/sdfctl_metadata.py` | v1 reader tests; dry-run no-op test |
| P1 | CI fixtures | `test/data/synthetic/`, `test/data/mini_bag/`, CI workflow file if present | build/unit/launch/fake saver/schema/replay |
| P1 | dependency manifest | `dependencies.repos` or docs manifest, `docs/MAPPING_DX_RUNBOOK.md` | `vcs import`, `rosdep install`, package-focused build |
| P1 | TTHW smoke script | `scripts/tthw_sdfctl_smoke.sh` | measures doctor/health/analyze wall time and exit codes |
| P2 | offline refine polish | `src/offline_refiner.cpp`, `scripts/sdfctl_offline_refine.py`, reports schema | replay determinism and `PROMOTION_ALLOWED` tests |
| P2 | promotion/rollback docs | `docs/MAPPING_DX_RUNBOOK.md`, `scripts/sdfctl_promote.py`, `scripts/sdfctl_rollback.py` | typed confirmation, rollback observe/disabled, verify-rollback |

### DX DUAL VOICES Consensus Table

| Dimension | Claude | Codex | Consensus |
|---|---|---|---|
| Overall stance | N/A due `deepseek-v4-flash` environment config | DX is currently fragmented but fixable with CLI/runbook/gates | N/A; no dual-model claim |
| Hardware boundary | N/A | MID-360 built-in IMU only must be first-class | N/A |
| First evidence | N/A | <=15 min realistic because bag/scenes require 2 min | N/A |
| Returning health | N/A | <5 min required and feasible through `sdfctl health` | N/A |
| Safety | N/A | fail-closed defaults and C1 gate prevent unsafe enforce | N/A |
| Docs | N/A | one runbook should replace scattered raw commands | N/A |

### Phase 3.5 Summary

Overall initial DX is 3.25/10 and planned DX is 8.4/10. TTHW targets are first evidence <=15 min and returning health <5 min. 13 Codex concerns are incorporated. Claude is unavailable due `deepseek-v4-flash` environment config. C1 remains the only unresolved User Challenge.

Cross-phase themes remain evidence gate, safety vs map cleanliness, deterministic artifacts, and operator-visible failure. The DX plan keeps normal competition navigation on reviewed static maps, makes dynamic filtering opt-in through observe/enforce gates, and avoids overbuilding hosted playgrounds, telemetry, GUI, semantic camera, independent electrical IMU/wheel odom, or public release work.

Phase 3.5 complete. Passing to Phase 4 final gate.
