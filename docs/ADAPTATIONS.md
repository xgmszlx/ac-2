# 平台适配清单（相对作者 ExplORB-SLAM 的**全部**改动）

原则：不动 SLAM 前端、不动 frontier 检测器、不动候选生成、不动运动约束。
以下每条都是为了"在这台机器上能跑"，且都写进了对应文件头注释。

## A. 依赖层（无 sudo）
| 项 | 处理 |
|---|---|
| octomap_ros / octomap_server / octomap_rviz_plugins / octomap / kobuki_msgs | `apt-get download` + `dpkg -x` 到 `localroot/`，`env.sh` 注入 `ROS_PACKAGE_PATH/LD_LIBRARY_PATH/CMAKE_PREFIX_PATH` |
| teb_local_planner / costmap_converter / mbf_* / libg2o | 同上（保住作者 launch 指定的 TEB，未降级 DWA） |
| python3-scipy/numba/scikit-learn/networkx/decorator/typish, libllvm8 | deb 解包 + `pip --target`；numba 0.48 可用 |
| ROS deb 的 cmake 配置写死 `/opt/ros/noetic` | 只把 `localroot` 内**确实存在**的路径重写（149 处），不存在的保留 |
| `sophus`（pip 无同名包） | `shims/sophus.py`：只实现作者用到的 SE3 `*`/`inverse`/`translation`/`rotationMatrix`，语义 `p_a = R_ab p_b + t_ab` |

## B. 机器人：kobuki → TurtleBot3 Waffle
原因（实测）：作者 kobuki URDF 的 `gpu_ray` 激光与 `type="depth"` 相机在本机 Gazebo 11 上
**永远不发布**（`/base_scan` 不注册；相机 topic 注册但零帧）。加 `always_on`、加 `<depth_camera/>`、
gpu_ray→CPU ray 全部无效。同为差速+RGB-D 的 Waffle 传感器在本机实测正常。
改动：`urdf/explorb_robot.urdf.xacro` + `urdf/explorb_waffle.gazebo.xacro`
- 相机光学帧改名 `camera_link_optical`（作者节点硬编码查这个帧）
- 深度/相机话题改 `depth_registered/*`（对齐作者 launch 的 remap）
- 分辨率 1920×1080 → 640×480
- 删掉 `<robotNamespace>`：带命名空间时 gazebo_ros 传感器插件只注册不发布（实测）

## C. World
原因（实测）：作者 world 的 `<physics default="0" name="default_physics">` 使其不是默认物理配置，
spawn 出来的机器人**所有传感器静默**（`/clock` 仍 1.0 倍实时推进、`pause: False`）。
改动：`worlds/loc_store.world` = 作者自己的 140 个模型实例（pose、`model://` URI 原样）+
sun + ground_plane + 标准 ODE 物理。**未改动任何模型几何或位姿**。

## D. 启动/参数
- 机器人 spawn 在**根命名空间**（插件命名空间问题见 B）；`move_base` 与决策器仍在 `robot_1`
  （作者的 action/service 名要求）
- costmap：`topic: /scan`、`sensor_frame: base_scan`、`robot_radius: 0.20`、`inflation_radius: 0.26`
  （作者 0.15/0.16 是为 kobuki）
- `cmd_vel` remap 到 `/cmd_vel`
- 不启动 rviz（无头计时跑，避免抢 CPU）；`GAZEBO_RESOURCE_PATH` 改为**追加**而不是覆盖
  （作者覆盖后 OGRE 找不到材质脚本）

## E. 挂具改动（不影响两臂公平性）
- `Controller.py`：`~strategy` 开关 + 决策审计 CSV；`~strategy_seed` 固定 `np.random`/`random`
  （作者候选采样与 Laplacian 锚点用全局 RNG，两臂都固定 → 更可重复）
- `WeightedPoseGraph.py`：`hallucinateGraph` 存一份 `last_plan`（**同一份** `make_plan` 结果，不额外规划）
- 作者 `show_debug_path` 参数实际是"是否计算栅格信息增益"，其 `else` 分支有签名 bug，
  因此两臂都固定 `show_debug_path=true`（作者实际使用配置）
