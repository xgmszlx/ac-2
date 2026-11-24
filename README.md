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

## 引用与许可

上游代码：Placed et al., *ExplORB-SLAM: Active Visual SLAM Exploiting the Pose-graph Topology*,
ROBOT 2022（GPL，见 `ws/src/orb_slam2_ros/LICENSE.txt`）。本仓库新增部分同样按 GPL 提供。
