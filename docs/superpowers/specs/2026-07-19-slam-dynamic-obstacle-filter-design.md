# SLAM 动态障碍过滤设计

日期：2026-07-19

状态：approved

目标平台：AMD Ryzen 5 4500U，6 核 / 6 线程，不依赖独立 GPU。

范围：`multiplenav_launch.py` 的多点同时 SLAM / 导航流程，以及 Gazebo 等价流程。

## 背景与根因

当前 Livox 点云转换为 `LaserScan` 后直接进入 `slam_toolbox`，中间没有动态 / 静态分类器。Nav2 的 STVL 可以对运行时 costmap 障碍做衰减，但无法可靠擦除已经写入 `/map` 并被 `StaticLayer` 使用的占据信息。

因此，动态障碍过滤必须发生在 `slam_toolbox` 之前。同时，Nav2 仍然使用实时、无延迟的过滤后点云进行避障，避免为了建图过滤而牺牲避障实时性。

## 目标

- 仅使用 Livox 以及现有 odom / TF，不引入额外传感器。
- 人、其他机器人等目标只要表现出运动，就被过滤。
- 动态身份在整个建图会话中保留，即使目标后来停止。
- Nav2 避障不受 SLAM 过滤延迟影响。
- 不在遮挡物后方伪造清空空间。
- 过滤器不健康时暂停 SLAM 并阻止保存地图。
- 离线前向 / 后向回放可清理后来才发现为动态目标的早期扫描。
- Gazebo 与真实环境均提供量化验证。

## 明确限制

- 仅凭 lidar 几何，无法识别从会话开始到结束都完全静止的人或机器人。
- 无法语义区分从未移动的箱子、机器人和固定设施。
- 不保证跨会话或永久稳定的身份 ID。
- 过滤器失败时绝不静默回退到原始 scan。
- STVL 不是 SLAM 动态障碍过滤的替代方案。

## 选定方案

采用实时 CPU-only 2D `LaserScan` 时间滤波，加离线最终地图精修。

拒绝以下方案：

- 仅离线处理：不能保护在线 SLAM 地图，也无法阻止不健康地图保存。
- 仅调参 / 仅 STVL：不能可靠清除 `/map` 中已经写入的动态占用。
- 稠密 3D / 神经网络方案：计算开销、依赖复杂度和可解释性不符合 R5-4500U 目标平台约束。

## 架构数据流

```text
/livox/lidar
  -> cpp_lidar_filter
  -> /livox/lidar_filtered
       |-> 分支 A：Nav2 STVL，直接消费，无等待
       |-> 分支 B：pointcloud_to_laserscan
              -> /scan_raw
              -> slam_dynamic_filter_node
                    - odom 运动补偿
                    - 约 10 帧 / 1 秒环形缓冲
                    - 线性 2D scan 分割
                    - 时间运动检测
                    - 轻量跟踪
                    - 会话动态记忆
              -> /scan_slam_filtered
              -> slam_toolbox
```

`slam_dynamic_filter_node` 还输出 tracks、dynamic mask 和 diagnostics。`safe_map_saver` 消费健康状态。

SLAM 数据允许约 1 秒延迟；Nav2 不等待该延迟。

## 包与组件

新增独立包 `slam_dynamic_filter`，不合并进 `cpp_lidar_filter`。

组件：

- `motion_compensator`
- `scan_segmenter`
- `dynamic_tracker`
- `scan_masker`
- `health_monitor`
- `safe_map_saver`
- `slam_dynamic_map_refiner`

ROS node 只负责 I/O 和参数。算法模块必须可在没有实时 ROS graph 的情况下单元测试。

## 运动补偿与帧有效性

使用带时间戳的 `odom -> base_link` TF。扫描端点在 `odom` 坐标系中比较，不在 `map` 坐标系中比较，以避免 SLAM 反馈污染动态判断。

以下情况拒绝该帧并暂停 SLAM 输出：

- 缺失 TF。
- TF 外推。
- odom 跳变。
- 时间戳反向。
- 有效 beam 过少。
- 有界队列溢出。

## 分割与跟踪

分割在约 720 个 beam 上做一次线性遍历。相邻端点距离超过阈值时切分 segment。1-2 点噪声段丢弃。

segment 特征包括：

- 质心
- 宽度
- 深度
- 线性度
- 角跨度

稳定长直线仅在运动补偿后仍在 odom 坐标系中保持空间稳定时作为墙体保护。随其他机器人平移的长直侧边不能被墙体保护吞掉；只要跨多帧表现出一致位移，仍必须进入动态候选。

人 / 机器人尺寸的独立 segment 成为动态候选。

不使用全局 3D DBSCAN、ICP 或神经网络。

复杂度为 `O(beams + tracks^2)`，track 硬上限为 32。

推荐初始参数：

- 缓冲：约 10 帧，0.8-1.2 秒。
- 匹配确认：至少 3 帧。
- 初始最小累计位移：0.15-0.25 m。
- 最小速度：0.12-0.20 m/s。
- 推荐建图速度：线速度不超过 0.8 m/s，角速度不超过 0.8 rad/s。

确认动态目标时，只接受连续多帧、在 odom 坐标系中发生、且不能由 TF 抖动、机器人转弯或 scan 边缘效应解释的运动。目标确认后，缓冲区内的早期 scan 被追溯 mask。

## 会话记忆

目标一旦确认移动，当前 track 在整个建图会话内保持动态，即使停止 30 秒以上也不恢复为静态。

系统支持遮挡期间的预测关联。休眠 track 可在附近重新激活。系统记录历史 beam 用于离线 mask。

该机制记忆的是 track 身份和 beam 历史，不是永久禁入空间。

## 安全 Scan 规则

| 情况 | 输出规则 |
| --- | --- |
| 确认动态回波 | 输出 `NaN` |
| 动态目标背后的未观测空间 | 输出 `NaN`，绝不伪造 free |
| 目标移动后真实墙体被重新观测 | 透传 range |
| 目标移动后真实 no-return | 透传 `inf` |
| 静态或未确认目标 | 原样透传 |
| 过滤器不健康 | 不发布帧，不回退原始 scan |

如果目标先静止后移动：在线过滤从确认后阻止未来插入；后续真实 free 观测可逐步清理残留；离线 replay 会追溯 mask 确认前 scan。

## ROS 接口

节点：`slam_dynamic_filter_node`

输入：

- `/scan_raw`，类型 `sensor_msgs/msg/LaserScan`
- `/tf`
- `/tf_static`
- `/Odometry`

输出：

- `/scan_slam_filtered`，类型 `sensor_msgs/msg/LaserScan`
- `/slam_dynamic_filter/tracks`，类型 `visualization_msgs/msg/MarkerArray`
- `/slam_dynamic_filter/dynamic_mask`，类型 `sensor_msgs/msg/LaserScan`
- `/slam_dynamic_filter/diagnostics`，类型 `diagnostic_msgs/msg/DiagnosticArray`
- `/slam_dynamic_filter/ready`，类型 `std_msgs/msg/Bool`

服务：

- `reset_session`，类型 `std_srvs/srv/Trigger`
- `freeze`，类型 `std_srvs/srv/SetBool`

v1 优先使用 `DiagnosticArray` key-values，不新增自定义消息。

`reset_session` 仅在机器人已停止且没有最终保存进行中时允许执行。`freeze` 停止 SLAM 输出，同时 diagnostics 继续发布。

每次新建图会话生成唯一 session ID。reset / restart 会使 ready 失效，并要求重新 warmup。`safe_map_saver` 拒绝 session ID 不匹配的 diagnostics、bag 或 map。

`/slam_dynamic_filter/ready` 是便捷状态信号，可使用 transient-local 或等价的启动安全行为，便于新订阅者获得当前状态；但它绝不能作为保存地图的唯一 gate。保存安全性以新鲜 diagnostics、session ID 和健康级别为准。

## Launch 与参数

`pointcloud_to_laserscan` 当前消费原始 `/livox/lidar`，必须改为消费 `cpp_lidar_filter` 输出的 `/livox/lidar_filtered`。

mapper 的 `scan_topic` 改为 `/scan_slam_filtered`。

Gazebo 使用同一套接口。

参数文件：

- `r5_4500u.yaml`
- `simulation.yaml`
- `low_load.yaml`

阈值不得硬编码。

## 建图会话 Bag

仅在 mapping mode 记录 bag。记录 topic：

- `/scan_raw`
- `/slam_dynamic_filter/dynamic_mask`
- `/slam_dynamic_filter/diagnostics`
- `/tf`
- `/tf_static`
- `/Odometry`

bag 使用压缩 MCAP 或 sqlite3，并按大小切分。启动前执行磁盘检查。低于预留空间时停止记录并阻止最终保存。

同时保存冻结参数 YAML、package / git revision 和 session metadata，并与 bag 关联。`/slam_dynamic_filter/tracks` 是 `MarkerArray` 可视化输出，不是机器可读 track stream，不要求记录。

离线 `slam_dynamic_map_refiner` 不读取单独录制的 tracks stream。它从 `/scan_raw`、`/tf`、`/tf_static` 和 `/Odometry` 使用完全相同的冻结参数快照确定性重算前向 / 后向 tracks，追溯标记早期 scan，生成清理后的 scan bag，使用相同 SLAM 参数回放，输出候选 `pgm/yaml`、质量报告和可复现元数据，且永不覆盖原始数据。录制的 dynamic mask 和 diagnostics 仅用于验证、对比和问题定位。

如果缺少重算所需数据、冻结参数、package / git revision 或 session metadata，或者无法完成确定性重算，offline final-map promotion 被阻止。

离线流程在建图结束后运行，最多使用 2 个 CPU 核心。

## 安全保存

移除无条件 30 秒最终地图自动保存。

`safe_map_saver` 保存地图必须满足：

- `ready=true`
- 新鲜 `DiagnosticArray` heartbeat，推荐最大年龄 500 ms，且该阈值参数化
- diagnostics session ID 与当前保存请求匹配
- diagnostics level 为 OK
- 连续 5 秒健康
- 无 backlog
- TF failure rate 低于阈值
- bag 与磁盘健康
- 已有足够 filtered scans
- map 非空
- 机器人低速或停止
- 收到显式 service call，或显式启用健康候选 snapshot

缺失、过期、session ID 不匹配或非 OK 的 diagnostics 均视为不健康并阻止保存。`ready` 只作为便捷状态参与判断，不能单独放行保存。

保存路径为：

- 在线候选 snapshot：`maps/sessions/<session-id>/snapshots/`
- 离线精修且人工 review 后的 final map：`maps/sessions/<session-id>/final/`

snapshot 作为 candidate 保存；final 只来自 offline-refined / reviewed 结果。

周期 snapshot 默认关闭，并且绝不自动提升为单导航默认地图。candidate 不自动 promotion。

## 失败矩阵

| 失败 / 状态 | 行为 |
| --- | --- |
| warmup | Nav2 正常工作，SLAM 等待 |
| 短暂 TF 失败 | 丢帧并暂停 SLAM |
| 重复 TF 失败 | 进入 error，阻止保存 |
| 队列过载 | 丢弃最旧等待数据，阻止延迟继续增长 |
| 目标过多 | 保留高置信目标，标记质量降级，阻止最终保存 |
| bag 满或记录失败 | 在线 SLAM 可继续，但离线完整性不可用，最终保存被阻止 |
| 过滤器崩溃 | Nav2 继续，SLAM 输入停止 |
| 崩溃后恢复 | 重新 warmup，不复用不可信中间 track |
| 紧急避障 | 仍由无延迟 STVL 处理 |

## R5-4500U 性能标准

在完整 Point-LIO + Nav2 + SLAM 环境中必须满足：

- 平均 filter 占用不超过单核 50%，约等于整机 CPU 8.3%。
- 峰值不超过单核 100%，且连续不超过 2 秒。
- 额外 RSS 不超过 200 MB。
- p95 单帧处理时间不超过 30 ms。
- p95 SLAM 数据延迟为 0.8-1.3 秒。
- 队列长度不超过 2 个确认窗口。
- 2 小时运行无崩溃、无泄漏。
- Nav2 避障增量延迟不超过 10 ms。
- 离线流程建图结束后运行，最多使用 2 个核心。

在线算法处理使用 1 个有界 worker，或 ROS 互斥 callback group 加有界队列，避免过滤器在 R5-4500U 上意外占用多核心。diagnostics 和 visualization 必须限频、非阻塞；发布失败或订阅端变慢不得反压核心 scan 处理路径。

负载削减顺序：

1. 10 Hz 降到 8 Hz。
2. 降低角分辨率。
3. 减少 tracks 数。
4. 减少可视化历史。
5. 切换 `low_load.yaml`。

禁止为了速度关闭 gates、原始 scan 回退保护或墙体安全保护。

## 测试

确定性单元测试覆盖：

- 运动补偿。
- 时间戳、TF、odom 跳变。
- segmentation，包括 `NaN`、`inf`、噪声。
- tracking，包括运动、停止、遮挡、交叉、重现、track cap。
- masking，包括不伪造 `inf` 和静态目标不变。

Gazebo 专用场景：

- 静态墙闭环。
- 人横穿。
- 移动后停止 30 秒。
- 静止 30 秒后移动。
- 墙前移动。
- 两个目标交叉。
- TF / CPU 故障注入。

真实测试矩阵：

- 人横穿、同向、对向。
- 停止 5 秒、30 秒、2 分钟。
- 两人。
- 另一台机器人。
- 墙、门、窄通道。
- 玻璃、反光、稀疏回波。
- 机器人直行、横移、旋转、组合运动。
- CPU 压力、TF 中断、低磁盘。
- 2 小时运行。
- 封闭安全区域。

## 定量地图质量

对比对象：

- 未过滤 baseline。
- 在线过滤结果。
- 离线最终结果。
- clean-reference 对齐栅格。

指标：

- 持久动态 false occupancy 降低不低于 90%。
- 固定墙误清除不超过 reference wall cells 的 1%。
- p95 确认时间不超过 1.2 秒。
- 静态有效 beam 被误判动态不超过 1%。
- unknown-area 增加不超过 3%。
- 真实重新观测后，在线残留不超过 5 秒消失。
- 先静止后移动目标的离线残留降低不低于 95%。

不得通过牺牲墙体清除安全来提高动态评分。

## 标定顺序

1. 静态 TF / 运动噪声。
2. segmentation 阈值。
3. 真实目标速度 / 噪声。
4. 位移、速度、帧数阈值。
5. 遮挡 / 重新关联。
6. 性能 profile。
7. 冻结参数后执行完整测试矩阵。

## 交付阶段

阶段 A：在线安全路径。内容包括 package、算法、wiring、diagnostics、gates、saver、单元测试、Gazebo 测试和性能验证。

阶段 B：R5-4500U bags 与标定。内容包括静态场景、单人 / 双人、机器人、冻结配置。

阶段 C：离线精修。内容包括前向 / 后向 mask、离线 SLAM、地图指标 / 报告和人工 promotion。

阶段 A 在没有阶段 C 的情况下也必须安全工作。

## 实现成功条件

- 真实环境和仿真使用相同接口。
- `slam_toolbox` 永远不接收原始 scan。
- Nav2 不被 SLAM 过滤延迟影响。
- 过滤器不健康时没有原始 scan 回退。
- 无条件最终自动保存已移除。
- R5-4500U 性能通过。
- Gazebo 故障和动态场景通过。
- 真实指标达标，或具备可复现实验证据解释未达标项。
- 运维文档明确说明限制。
