# 固定地图纯定位（run_loclite_online）

该模式使用 GenZ-LIO 的 ESIKF（混合度量更新）跟踪已有的 PCD 地图。入口、命令行、核心输出 topic、TF、状态编号以及冷启动匹配流程与 [Super-LIO 的纯定位模式](https://github.com/Aecric/Super-LIO_RSLiDAR/blob/ros2/docs/localization.md)（`run_loclite_online`，接口参照 hikari_loclite）一致；建图仍使用原有 `genz_lio_node`。纯定位只把固定地图建成一次体素地图：运行期间不融合新扫描、不按 `map_range` 裁剪、不保存地图。仅支持 ROS 2。

## 构建与运行

```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build --base-paths ros --packages-select genz_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 run genz_lio run_loclite_online \
  --config "$(ros2 pkg prefix --share genz_lio)/config/loclite_livox.yaml" \
  --map_path /path/to/map_dir
```

冷启动依赖的 KISS-Matcher 与 GenZ-ICP 已 vendored 在 `ros/thirdparty/`（与 Super-LIO 同一份）；首次 CMake 配置会从 GitHub 下载 KISS-Matcher 的依赖 ROBIN（含 PMC / xenium），需要联网。CMake ≥ 4 的主机上，PMC 的下载子工程需要 `export CMAKE_POLICY_VERSION_MINIMUM=3.5`。GenZ-ICP 自带 Sophus，tsl::robin_map 则复用 GenZ-LIO 核心 `cpp/genz_lio/3rdparty/tsl` 的那份。

`--config <yaml>`、`--config=<yaml>`、`--map_path <dir>`、`--map_path=<dir>` 均支持，配置参数放在 `--ros-args` 之前；`--ros-args -p <参数>:=<值>` 可覆盖配置文件里的任意参数。地图目录应包含 `global.pcd`。

```bash
ros2 launch genz_lio loclite.launch.py \
  config:=/path/to/loclite_livox.yaml \
  map_path:=/path/to/map_dir
```

地图路径优先级：CLI / launch 的 `map_path` → 配置 `system.map_path` 下的 `global.pcd` → `fixed_map.global_pcd`。相对路径按当前工作目录解析，部署建议使用绝对路径。地图不存在、为空或加载失败时进程非零退出。

配置是 **GenZ-LIO 自己的 YAML 格式**（与 `config/default/*.yaml` 相同，`common / runtime / preprocess / mapping / ... / noise_model`），通过 `config_path` 参数交给节点（`--config` 即设置它），不是 `ros__parameters` 文件。定位接口的键与 Super-LIO 的 `loclite_livox.yaml` 同名，只是写成嵌套形式：

| Super-LIO（平铺） | 本包 YAML（嵌套） |
|---|---|
| `system.map_path` | `system: map_path` |
| `common.map_frame_id` | `common: map_frame_id` |
| `fixed_map.global_pcd` | `fixed_map: global_pcd` |
| `runtime.publish_tf` / `runtime.imu_timeout_sec` … | `runtime: publish_tf` …（与核心的 `runtime: max_threads` 同段） |
| `reloc.fc.max_rmse` … | `reloc: fc: max_rmse` … |

YAML 中的值作为这些 ROS 参数的默认值，因此 `--ros-args -p reloc.origin_prior.x:=2.0` 等覆盖方式与 Super-LIO 完全一样（`system.map_path` 也是这样由 `--map_path` 设置的）。里程计参数（话题、`preprocess.*`、`mapping.*` 外参 `extrinsic_t / extrinsic_r` 为 `p_imu = R * p_lidar + t`、噪声等）沿用 GenZ-LIO 的 mid360 配置（`tools/configs/genz_mid360.yaml`：官方模板 + Mid-360 外参平移）；`publish` / `pcd_save` 段对定位节点无效。

固定地图先按 `fixed_map: voxel_size`（默认 0.1 m）降采样，每个点赋 `fixed_map: point_cov`（默认 0.0004 m²）的各向同性协方差，再一次性建成 GenZ-LIO 的八叉树体素地图（`mapping: voxel_size` 为根体素，平面点与点到点候选都来自这些地图点）。

## 初始化与恢复

与 Super-LIO 相同：设备**静止**时累积 10 帧点云（IMU 系）与至少 20 个 IMU 样本，用平均加速度把窗口拉平，对每个候选位姿执行 NDT 粗配 → GenZ-ICP 精配 → **FC 假阳性屏障**，通过后开始跟踪。候选来源：

1. **`/initialpose`**（RViz 2D Pose Estimate）：给出 **map ← lidar_frame** 初值，随时可发，会打断正在进行的冷启动或当前跟踪。
2. **冷启动**（没有初值时自动进行：启动、LOST 之后、重新启用之后）：
   - 先假设设备在**地图原点（建图起点）**附近：在 `reloc.origin_prior.{x,y,z}` 按 `yaw_step_deg`（默认 45°，共 8 个）枚举 yaw 假设；
   - 原点假设都未通过 FC 时，再用 KISS-Matcher 对整张固定图做无先验全局配准（Quatro 4-DoF，每个 `reloc.kiss.voxel_sizes` 出一个候选）。

FC 的各项阈值、对称歧义判定（多个通过的候选相距 > 1 m 或 > 10° 时整个窗口放弃）、KISS 需要 `min_agreeing_candidates` 个一致候选、`reloc.cold_start.max_attempts`（0 = 不限）等规则和日志前缀 `[FC-Init]` 都与 Super-LIO 一致，代码在 `ros/src/loclite/`（与 se3-lio fork 共用同一份）。

验证通过后滤波器从该位姿重新启动：GenZ-LIO 用随后的 IMU 样本重新初始化（> 20 个样本，约 0.1 s，设备仍需静止），重力方向随初始位姿转到地图系，位姿协方差设为 0.01。与 `genz_lio_node` 一样，输入回调与扫描处理在两个 executor 上，配准阻塞期间收到的数据在 `ScanBuffer` 中排队，按时间顺序继续处理。

平面 + 点到点有效匹配 < 10、ESIKF 更新被回滚或位姿非有限时转 LOST（日志 `[Track] lost`），停止发布并重新进入冷启动；IMU / 雷达掉线超过 `runtime.imu_timeout_sec / lidar_timeout_sec`（默认 1 / 2 s）同样转 LOST。

`~/set_active`（`std_srvs/srv/SetBool`）停用/启用传感器订阅；节点名默认为 `genz_lio_loc`（`ros2 run` 与 launch 相同），即 `/genz_lio_loc/set_active`，不会与建图节点 `genz_lio` 冲突。LiDAR / IMU 时间戳倒退（如 bag 循环播放）时缓冲区清空，定位同样从头开始。重新启用后重新冷启动或等待 `/initialpose`，固定地图保留。

## 输出约定

与 Super-LIO 纯定位相同：

| Topic | 类型 | 内容 |
|---|---|---|
| `/hikari_loc/odom` | `nav_msgs/msg/Odometry` | map ← lidar_frame；线速度在 lidar_frame 中表达 |
| `/hikari_loc/path` | `nav_msgs/msg/Path` | map 下雷达轨迹；最小间隔 0.1 s，最多 5000 点，重新初始化时清空 |
| `/pcdmap` | `sensor_msgs/msg/PointCloud2` | map 下固定地图，可靠、transient-local、深度 1，支持晚订阅 |
| `/hikari_loc/loc_state` | `std_msgs/msg/Int32` | 与 hikari_loclite 同一枚举编号 |
| `/hikari_loc/ndt_status` | `std_msgs/msg/Int32` | 固定 0 |
| `/hikari_loc/loc_status` | `visualization_msgs/msg/Marker` | 定位状态文字 |
| `/hikari_loc/status` | `std_msgs/msg/Float32MultiArray` | `[state, ndt_conf, imu_age_s, lidar_age_s, fps, in_map]` |

状态编号：0 未初始化、1 初始化中、2 GOOD、3 DEGRADED、4 LOST、5 WAIT_FOR_INITIALPOSE；本实现使用 1/2/4/5。`ndt_conf` 固定 -1（不可用）。状态以 5 Hz 发布。

TF：`map → base_frame`（`system.base_frame_id`，默认 `livox_frame`，非雷达时按 `system.base_to_lidar_*` 换算）以及 `livox_frame → level_frame`（去掉 roll/pitch、保留 yaw）。TF 与 odom 使用雷达帧末时间戳，不做 IMU 高频外推。

## 验证

合成地图集成测试（与 Super-LIO 的 `localization_smoke.py` 相同用例：CLI 错误处理、晚订阅地图、等待初值、`/initialpose` 初始化、原点先验冷启动、位姿/TF/状态接口、重复初值、停用/启用、看门狗、地图文件不变）：

```bash
ROS_DOMAIN_ID=173 python3 ros/tests/test_localization_smoke.py
```

真实数据（2026-10-22，Mid-360 斜装约 49°，`mapping_1784793089`，200 s / 104 m）：以 Super-LIO 对同一 bag 建的 0.1 m 地图作为 `global.pcd`，实时回放 bag、不发 `/initialpose`。原点先验冷启动约 1 s 内通过 FC（8 个 yaw 假设中 1 个 PASS），之后全程 GOOD、无 LOST、无 > 0.5 s 输出间隔；相对 Super-LIO 建图轨迹（换算到 IMU 位置）位置误差中位 / p95 / 最大为 1.0 cm / 2.0 cm / 9.7 cm，yaw 平均 0.09°、最大 0.78°；约 46% CPU、511 MB RSS。把 `reloc.origin_prior.x` 设到 15 m 时原点假设全部被拒，KISS-Matcher 给出 3 个一致候选并通过 FC（约 2 s）。注意地图与回放来自同一次采集，跨天、场景变化后的精度和 FC 阈值仍需用实际地图 + bag 复核。
