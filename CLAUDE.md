# pfa-nav — Claude 工作上下文

> RoboMaster 哨兵机器人 ROS2 导航栈。fork 自 `grow-up-happily/pfa-nav`（厦门理工学院战队），上游基于 `SMBU-PolarBear-Robotics-Team` 的"北极熊导航"。
> 架构本质：**一个 colcon 工作空间 + 一组根级 shell/python 调度脚本**。

---

## 一句话架构

`点云雷达(Livox MID360) → point_lio (LIO+SLAM) → small_gicp 重定位 → nav2 (规划+控制) → /cmd_vel → 串口下发底盘`，仿真用 Gazebo Ignition (`rmu_gazebo_simulator`) 替换硬件层。

## 数据流(运行时)

```
            ┌──────────────────────── 仿真 / 实车（二选一）─────────────────────┐
            │                                                                  │
   Gazebo  ─┤  /livox/lidar  /livox/imu                                       │
   实车驱动 ─┘  ↓                                                              │
                point_lio ──────────► /scan(2D) + /cloud_registered(3D) + tf(odom→base_link)
                  │
                  ├── slam=True   →  保存 scans.pcd + 2D 栅格(.pgm/.yaml)
                  └── slam=False  →  small_gicp_relocalization 拿先验 PCD + map.yaml 做重定位
                                          ↓
                                       nav2 (planner + controller + bt_navigator)
                                          ↓
                                       /cmd_vel  →  send.py / game.py  →  串口 (S{...}E)
```

**所有 topic / node / action 都加 namespace** `/red_standard_robot1/...`（订阅外部话题必加 remap，例：`--ros-args -r __ns:=/red_standard_robot1`）。

---

## 模块映射（src/ 下 21 个 ROS package）

| 模块路径 | 角色 |
|---|---|
| `pb2025_sentry_nav/pb2025_nav_bringup` | **核心 launch + config + map 仓**。所有 launch 入口在此（`rm_navigation_simulation_launch.py` / `rm_navigation_reality_launch.py` / `slam_launch.py` / `localization_launch.py` / `navigation_launch.py`）|
| `pb2025_sentry_nav/point_lio` | LIO+SLAM 主算法，**writeBinary 在 main() 末尾**，必须 SIGINT 走完清理才能落 PCD |
| `pb2025_sentry_nav/small_gicp_relocalization` | 把当前点云配准到先验 PCD，初始化需 RViz `2D Pose Estimate` |
| `pb2025_sentry_nav/livox_ros_driver2` | Livox MID360 驱动 |
| `pb2025_sentry_nav/loop_closure_3d` | 回环检测（**当前已禁用**，README"取消地图自动匹配和回环") |
| `pb2025_sentry_nav/fake_vel_transform` | x11 显示问题已取消该 fake_vel 节点 |
| `pb2025_sentry_nav/{pb_omni_pid_pursuit_controller, pb_nav2_plugins, pb_teleop_twist_joy}` | 自定义 nav2 控制器/插件、手柄 |
| `pb2025_sentry_nav/{terrain_analysis, terrain_analysis_ext, sensor_scan_generation, pointcloud_to_laserscan}` | 点云预处理流水线 |
| `pb2025_sentry_nav/loam_interface` / `ign_sim_pointcloud_tool` | 仿真适配层 |
| `rmu_gazebo_simulator/rmu_gazebo_simulator` | Gazebo Ignition 仿真世界 + 机器人 |
| `rmoss_core/{rmoss_base, rmoss_cam, rmoss_projectile_motion, rmoss_util}` | RoboMaster 通用基础设施（OSS 上游）|
| `rmoss_gazebo/{rmoss_gz_base, rmoss_gz_cam, rmoss_gz_bridge, rmoss_gz_plugins}` / `rmoss_gz_resources` | Gazebo 桥接层、资源 |
| `rmoss_interfaces` / `auto_aim_interfaces` / `pb_rm_interfaces` | 自定义 .msg/.srv/.action 接口包 |
| `pb2025_robot_description` | 机器人 URDF/xacro，**雷达姿态 `<xmacro_block pose=...>` 在此**|
| `m-explore-ros2/{explore, map_merge}` | 自主探索（多机协同时用）|
| `hero_lidar` | 英雄机型雷达包（用于 hero_to_sentry 转图场景）|
| `wp_map_tools` | 航点编辑（`wp_saver` / `add_waypoint_*.launch.py`）|
| `joint_state_publisher{,_gui}` | URDF 关节状态发布（GUI 调试用）|
| `small_gicp` | GICP 配准库（也需用 cmake 单独 `make install` 到系统）|
| `sdformat_tools` / `pb2025_robot_description` | xacro/sdf 工具链 |

---

## 关键 schema / 路径硬编码（动代码前必查）

| 写死的位置 | 内容 | 改了会影响 |
|---|---|---|
| `slam.sh` L25-30 | `NAMESPACE=red_standard_robot1`、`MAP_SAVE_DIR=src/pb2025_sentry_nav/point_lio/PCD`、`MAP_NAME=scans` | map_saver 服务路径、PCD 备份路径 |
| `nav.sh` L10-13 | 同上 + `GAME_PCD_DIR=src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality` | 启动前自动 `cp scans.pcd → game.pcd` 作为先验 |
| `localization_launch.py` L129 | `pcd_save.pcd_save_en: True`（**写死**，纯导航也写 PCD）| 实车跑 nav.sh 时也会持续刷 PCD |
| `pb2025_nav_bringup/config/reality/nav2_params.yaml` | `gravity` / `gravity_init` 三元组 | **改雷达 xacro RPY 必须同步改这两个**，否则 LIO 漂 |
| `pb2025_nav_bringup/config/reality/mid360_user_config.json` | Mid360 实车 IP | 换雷达硬件需改 |
| `pb2025_nav_bringup/map/{reality,simulation}/` | `game.{pgm,yaml}`、`rmuc_2024/2025/2026.yaml`、`rmul_2024/2025.yaml` | nav2 加载的目标场地图 |
| `waypoints.yaml` (根目录) | `Waypoints_Num` + `Waypoint_1..N`（pose 列表）| `game.py` / `wp_saver` 读写格式 |
| `game.py` L38 | `nav_ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')` | **实车版无 namespace**；仿真用 `/red_standard_robot1/navigate_to_pose` |

**串口协议**（`send.py` + `game.py`）：
- send.py：`b'S' + struct.pack('<ffB', vx, vy, status) + crc8 + b'E'` (二进制)
- game.py README 描述：`S{x_sign}{x_vel:03d}{y_sign}{y_vel:03d}{status}E` (ASCII)
- **两者协议不一致**——实际跑哪个看场景；动串口前看清当前用的是哪个 sender

---

## 入口脚本（根目录，全部用 `SCRIPT_DIR` 自适应路径，跨机可移植）

| 脚本 | 作用 | 关键 trap 设计 |
|---|---|---|
| `slam.sh` | 建图，仿真 + 实车通用 | trap SIGINT → 先 `map_saver service call`（必须趁 lifecycle active）→ 再 SIGINT launch（point_lio 走完 writeBinary）→ 验证 PCD mtime → 时间戳备份 |
| `nav.sh` | 实车纯导航 | 启动前 `cp scans.pcd → game.pcd`，trap 同上 |
| `save.sh` / `save_map_timestamp.sh` | 手动保存 2D 地图 | save.sh 覆盖 `game.{pgm,yaml}`；timestamp 版不覆盖 |
| `kill_ros.sh` | 清残留进程 + DDS 共享内存 | 三阶段 INT(5s)→TERM(3s)→KILL，**永不杀** vscode/code/jetbrains/colcon/dbus/systemd 及自身祖先链 |
| `build.sh` | colcon 限并发 2 编译 + 启动实车建图（一条龙）| 内存 < 16GB 必用 `--parallel-workers 2` |
| `auto_align_map.py` + `hero_to_sentry_map_converter.py` | 旧地图重映射到新坐标系（**只做 yaw + xy 平移**）| 假设前提：**point_lio 重力对齐**（RPY 中 roll/pitch 已被 SLAM 消化），不要试图加回 RPY |

---

## 关键 pitfalls（这里掉坑过、要记住）

1. **point_lio PCD 落盘机制**：`writeBinary` 在 `main()` 末尾，必须 `SIGINT` 让进程走完析构。`SIGKILL` / 超时杀 = 丢图。
   - launch 必带 `sigterm_timeout:=30 sigkill_timeout:=60`（slam.sh/nav.sh 已默认带）
   - PCD mtime 不更新提示 → 调到 60/90s

2. **map_saver 必须在 launch SIGINT 之前调**：lifecycle 一旦 deactivating，service call 立刻 timeout。slam.sh cleanup 顺序: ① save_map ② kill -INT launch（顺序写反 = 丢 2D 地图）

3. **改雷达姿态(xacro RPY)必同步改 nav2_params.yaml `gravity`/`gravity_init`**：两者描述同一物理事实（重力方向），不一致 → LIO 启动就漂

4. **重力对齐 SLAM 下旧图不用重建**：用 `auto_align_map.py --apply` 自动对齐到新坐标系（dyaw + dx + dy），**别用 RPY/forward_gravity/gravity_yaw 这些旧模式**（已被设计性废弃，会导致 2D/3D 错位）

5. **GICP 重定位需手动初始化**：实车启动 nav.sh 后必须在 RViz 发 `2D Pose Estimate`，否则 `[small_gicp_relocalization]: GICP did not converge` 一直刷

6. **仿真 namespace 不同**：仿真节点全在 `/red_standard_robot1/...`；实车 game.py 写的 `/navigate_to_pose` 是无 namespace 版（**搬到仿真要加 namespace 否则连不上 action server**）

7. **回环检测和 fake_vel 节点已禁用**：见近期 commit。loop_closure_3d、fake_vel_transform 包还在 src/ 但不参与运行时；调试 odom 漂移别去找它们

8. **colcon 内存爆炸**：< 16GB 必加 `--parallel-workers 2`，否则编 small_gicp / point_lio 时 OOM

9. **libusb 冲突 point_lio 起不来**：`LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0` 写进 `install/local_setup.bash` 末尾

10. **小心两份串口协议**：send.py 是 `<ffB`+CRC8；game.py 注释说是 ASCII `S{sign}{vel:03d}...E`。动串口前先 grep 确认当前用的是哪边

11. **上游路径硬编码**（已在本机修，但 git merge upstream 后会被覆盖）：上游开发者把 `/home/tompig/pfa-nav-main/...` 这类绝对路径写死在以下位置，每次同步上游后**必须手动重新改回相对路径**：
    - `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_simulation_launch.py:227`
    - `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py:231`
    - `HERO_MAP_CONVERTER_README.md` 文档示例（仅注释，不影响运行）

12. **save.sh 在 nav 模式下不可用**：`map_saver_server` 只在 `slam:=True` launch 启动（slam_launch.py L40 lifecycle）。`localization_launch.py` 只起只读的 `map_server`。在 `slam:=False` 模式下调 save.sh 会 timeout 报错，这是预期行为（修复后会清晰提示"is slam:=True running?"）

13. **`auto_save_map=True` 时 periodic_map_saver 节点的 IfCondition**：`slam=True AND auto_save_map=True` 才启动（rm_navigation_*_launch.py L219-223）。如果发现没有 `auto_map_<ts>.{pgm,yaml}` 中间快照生成，先确认这两个 launch arg 是否都为 True

---

## 本机相对 upstream 的修改（2026-05-06 由 Claude 修）

git merge upstream 时这几处可能产生冲突，**保留本机版本**，不要用上游：

| 文件 | 上游 → 本机 | 修复理由 |
|---|---|---|
| `save.sh` L10-19 | `map_saver_cli -f game`（漏 namespace + 静默失败）→ `service call /red_standard_robot1/map_saver/save_map`（带 timeout + 错误处理）| 跟 `slam.sh` / `save_map_timestamp.sh` 实现风格对齐；上游版本在带 namespace 的项目里**完全不工作** |
| `rm_navigation_simulation_launch.py:227` | `/home/tompig/pfa-nav-main/...`（上游开发者绝对路径）→ `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation`（相对项目根 cwd）| `os.makedirs('/home/tompig/...')` 在普通用户机上抛 `PermissionError`，导致 periodic_map_saver 崩溃，丢失中途快照 |
| `rm_navigation_reality_launch.py:231` | 同上（reality 路径）→ 相对路径 | 同上 |
| `auto_nav.py` L1029-1140（drive 阶段）| 边走边转，初始角差大时无法对准 → drive 阶段加 yaw-gate，超阈值切回 align（停→对→走）| 实测 drive 中底盘被推偏后，原版只能边走边修正，14 秒前进 0.43m + 横移 1.5m 触发超时 |
| `auto_nav.py` L1265-1267（is_calibration_timeout）| 计算超时时间 → 直接 `return False` 禁用 | 用户要求在调试期不让超时回退兜底，便于观察 yaw-gate 行为 |
| `auto_nav.py` 新增 `--enable-extra-straight` / `--extra-straight-wait-seconds`（默认关闭） | 新增 4→5 单向额外直线段（停→对→走）+ 到 5 后静止 5s（每 0.1s 发 zero Twist + angle_diff=0.0）+ 跳过 5 直接 nav2 到 order 中下一个点 | 不动 STRAIGHT_WAYPOINT_IDS=(2,3)/BASE_BEGIN_ID=5；新增 `CALIBRATION_MODE_EXTRA_STRAIGHT` + `EXTRA_STRAIGHT_TRANSITIONS={(4,5)}`（单向）+ `extra_straight_wait_loop` 镜像 base_wait_loop 但跳过 correction |

---

## 文档资源（详细参考用，不必每次重读）

- `README.md` — 主文档（一键脚本、4 种使用场景、实车配置、bag 录制、LIO 调参、FAQ）
- `HERO_MAP_CONVERTER_README.md` — auto_align_map + hero_to_sentry 工具完整手册（426 行，包含算法/CLI/排错）
- `src/pb2025_sentry_nav/pb2025_nav_bringup/{launch,config,map,rviz,behavior_trees}/` — nav2 全套配置在此

## 上游同步

```bash
git fetch upstream && git merge upstream/main
# 或 rebase: git rebase upstream/main
```

`upstream` = `grow-up-happily/pfa-nav`（厦理工战队 fork），`origin` = `moyunliuyin/pfa-nav`（你的 fork）。
