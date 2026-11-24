# ExplORB-SLAM 定位可观性并行线（locobs line）

主动 SLAM 并行线：以开源 [ExplORB-SLAM](https://github.com/JulioPlaced/ExplORB-SLAM) 为闭环基线，
研究**候选终点有探索价值、但机器人途中遭遇动态遮挡导致定位几何约束持续不足**（失跟/重定位/任务延误）这一问题。

## 现在到哪一步了

- ✅ 独立 worktree / 独立 ROS+Gazebo 端口 / 独立日志目录；全栈在无 sudo 条件下编译通过
- ✅ 相机 + 深度 + 激光 + 里程计在 Gazebo 侧稳定发布（换机器人、重建 world、去命名空间后打通）
- ✅ 方法代码已实现并与基线共用同一候选池（可观性估计 / 遮挡预测 / 新特征先验 / 有界亏空代价）
- ❌ **尚未**跑出第一轮 A/B 数据：全栈一起启动时 `gzserver` 会段错误（已定位为启动时序问题）

详细状态：[`docs/STATUS_20261124.md`](docs/STATUS_20261124.md)
平台适配与动机（每条都有 A/B 证据）：[`docs/ADAPTATIONS.md`](docs/ADAPTATIONS.md)

## 目录

| 路径 | 内容 |
|---|---|
| `upstream/` | 作者原始文件快照（未改），用于对照 diff |
| `ws/src/orb_slam2_ros`, `frontier_detector`, `decision_maker`, `robot_description`, `kobuki_plugins` | 作者的 5 个包（改动全部记录在 `docs/ADAPTATIONS.md`） |
| `ws/src/explorb_scenarios/` | 本线挂具包：仿真 bring-up、确定性动态演员、记录器、预算守卫、world/URDF/param 适配 |
| `ws/src/decision_maker/src/python/loo_*.py` | 方法实现（可观性 / 遮挡场 / 有界代价 / 决策审计） |
| `shims/sophus.py` | 依赖适配层（SE3 最小实现） |
| `tools/` | 传感器探针、GLX 探针、资产拉取脚本文档 |
| `results/`, `logs/` | 逐任务输出与构建/运行日志 |

## 快速开始

```bash
# 1) 取回不进 git 的两份大资产（ORB 词袋 48MB + AWS Gazebo 模型 ~190MB）
./tools/fetch_assets.sh

# 2) 环境（无 sudo 本地前缀，独立端口）
source ./env.sh
source ./ws/devel/setup.bash          # 首次需先 catkin_make
export ROS_MASTER_URI=http://localhost:11331 GAZEBO_MASTER_URI=http://localhost:11346

# 3) 构建
cd ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j12 && cd ..

# 4) 传感器体检（应看到 rgb/depth/scan/odom 均非零）
python3 tools/sensor_probe.py 8

# 5) 一次完整计时任务
./run_task.sh native static task_001 90 0
```

## 两臂（同一候选池、同一运动约束、只有打分不同）

| strategy | 含义 |
|---|---|
| `native` | 作者原生 ExplORB（信息增益 argmax） |
| `travel_only` | 控制臂：作者增益 + 旅行代价 |
| `loo_full` | 完整方法 |
| `loo_notemporal` | 消融：去掉动态遮挡预测 |
| `loo_noduration` | 消融：去掉持续时间项 |

## 2026-11-25 新增（定位可观性线）

| 位置 | 作用 |
|---|---|
| `build.sh` | 可复现构建。`source ws/devel/setup.bash` 会**覆盖** `ROS_PACKAGE_PATH` 与 `CMAKE_PREFIX_PATH`，且 catkin 的 `generate_messages` 只在带 `.catkin` 标记的前缀里找 msg-paths —— 脚本把这两个坑都处理了（详见脚本内注释） |
| `ws/src/visual_query/` | 纯 C++（仅 Eigen）局部视觉查询：80–150 点快照下采样、常速度动态预测、四因子可微权重 `w_fov·w_stat·w_dyn(t)·w_rel`、`query(pose,time)` / `query_pair(...)`。自带 27 项离线自检 |
| `ws/src/teb_local_planner/` | `noetic-devel` 源码 + 两条有界软代价边 `EdgeVisualObservability`（一元，按暴露时间加权）与 `EdgeCoVisibility`（二元，固定感知间隔 0.45 s，共视+视差）。权重默认 0 = 作者图。自带 11 项离线自检 |
| `ws/src/explorb_scenarios/scripts/dynamic_mask.py` | **只用传感器**的动态像素掩码（自运动补偿的深度残差）+ 测量得到的动态物体（中心/尺寸/常速度） |
| `ws/src/explorb_scenarios/scripts/slam_export.py` | 只读导出：静态地图点（id/3D/观测数）、关键帧、边、位姿+状态、内外参、静态栅格 |
| `ws/src/explorb_scenarios/scripts/goto_goal.py` | 固定起终点目标下发（相对 agent 启动时刻调度） |
| `tools/eval_run.py` `tools/ab_summary.py` `tools/run_ab.sh` | 离线评测（ATE/RPE/覆盖率/失跟/TEB 耗时）与 4 臂 A/B 驱动 |
| `tools/selfcheck_loo.py` `tools/correlation_mask_n.py` | 无 Gazebo 自检；预测遮挡 vs 实际可用观测的相关性日志 |
| `docs/STATUS_20261125.md` `REPORT_EXPLORB_TEB.md` | 逐轮状态（含每一次失败的根因）与总报告 |

一键自检（不需要 Gazebo / master）：

```bash
python3 tools/selfcheck_loo.py                       # 13 项
./ws/devel/lib/visual_query/selfcheck_visual_query    # 27 项（catkin 构建后可用）
./ws/devel/lib/teb_local_planner/selfcheck_visual_edges  # 11 项
```

## 引用与许可

上游代码：Placed et al., *ExplORB-SLAM: Active Visual SLAM Exploiting the Pose-graph Topology*,
ROBOT 2022（GPL，见 `ws/src/orb_slam2_ros/LICENSE.txt`）。本仓库新增部分同样按 GPL 提供。
