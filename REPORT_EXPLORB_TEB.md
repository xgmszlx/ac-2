# REPORT_EXPLORB_TEB — 动态遮挡下保持视觉约束连续性的 TEB 局部轨迹优化（原型）

> 本报告只写**实际跑过、有日志/CSV 为证**的东西。凡未完成的，明确写"未完成 + 卡在哪 + 下一步"。
> 分支 `research/explorb-locobs-20261124`，工作区 `/home/wcqw/explorb_line_20261121`。
> 逐轮细节与证据见 `docs/STATUS_20261125.md`（滚动更新）。

---

## 0 一句话结论

**原生探索闭环已打通并在真仿真里跑通**（frontier → Filter → goal → TEB → `cmd_vel` → 机器人移动 → 地图更新，
一轮 240 sim-s 里 64 个 frontier 目标、63 次到达、机器人走 8.85 m、覆盖率 35.1 m²、ATE 0.145 m）。
**P2（统一动态掩码接口 + 只读导出）、P3（两处口径修正 + 纯 C++ 视觉查询模块）已完成**，
其中 Python 侧 13 项、C++ 查询 27 项、TEB 两条 g2o 边 11 项离线自检全部通过。
**P4 的两条软边已经编译进插件并在真跑里进入 g2o 图**（每次运行的
`visual_edges=/covis_edges=` 计数可核对），四项自检中两项过、两项给出明确失败结论；
**P5 的 4 臂 A/B 跑完，四臂在固定起终点上全部到达**（ATE 0.024–0.049 m、全程跟踪），
但两个诚实结论必须同时说：
① 加了视觉代价后 TEB 单次求解从 **74.7 ms → 126–172 ms**，10 Hz 控制周期在任何一臂都没满足；
② 在 λ=1 下轨迹几乎不变（路径差 <2%），**本轮数据不支持"方法更优"**——需要先做 λ 标定与解析雅可比。
优化前的同一项代价曾高达 **1857 ms**（单次最坏 28.7 s），代价结构与三步优化见 §6.3。

---

## 1 底座与公平性（不许动的部分）

| 约束（任务书） | 本轮的落实 |
|---|---|
| 保持 ExplORB-SLAM / ORB-SLAM2 / TEB 底座不变，禁止迁 ORB-SLAM3 / NGD-SLAM | 未引入任何新 SLAM；TEB 仍为 `teb_local_planner`（§5 说明是 deb 还是源码包） |
| 禁改 KF 插入 / 匹配接受 / 局部建图 / BA 内核 | ORB-SLAM2 的唯一改动是**特征提取入口**的动态掩码过滤（`Frame::ApplyDynamicMask`），在 `ExtractORB` 之后、任何匹配/KF/BA 之前执行；掩码为空时行为与作者逐字节相同 |
| 禁做离散动作选择器 / 失跟恢复控制器 / 新网络 | 全仓库无 WAIT/LEFT/RIGHT 类动作选择、无新模型 |
| Gazebo GT 只用于离线评测 | `slam_export.py` 的 `manifest.json` 写死 `ground_truth_online: false`；在线链路里没有任何 `model_states`/GT 订阅；GT 只被 `tools/eval_run.py` 读 |
| 两臂共享完全一致的感知底座 | 同一个 `dynamic_mask.py` 掩码、同一个 costmap、同一份候选池、同一段固定起终点路线；四臂**只差 TEB 的两个权重** |

---

## 2 阶段一：闭环与公平感知底座

### 2.1 `Filter.py` 输出 0 条的根因（这一节是本轮最大的发现）

前几轮的结论是"Filter.py 的过滤条件太严"，实测证明那是**打偏的**：Filter 的 6 个过滤条件
在修好之前**一次都没有被执行过**。真正的因果链是四层，逐层剥开：

| # | 根因 | 证据 | 修复 |
|---|---|---|---|
| ① | `run_task.sh` 先 `source env.sh`（把 no-sudo 本地前缀加进 `ROS_PACKAGE_PATH`），随后 `source ws/devel/setup.bash` —— catkin 的 setup.bash **整体覆盖** `ROS_PACKAGE_PATH`，本地前缀被抹掉，`teb_local_planner` 不可见 → move_base 启动即 FATAL | `rospack find teb_local_planner` → not found；`[FATAL] Failed to create the teb_local_planner/TebLocalPlannerROS planner … Declared types are base_local_planner/TrajectoryPlannerROS dwa_local_planner/DWAPlannerROS` | setup.bash 之后重新前置本地前缀 + 硬检查 |
| ② | move_base 死了 → 无 global costmap publisher、无 `make_plan` 服务 → `Filter.py` 卡在 `SimpleRobot.__init__` 的 `rospy.wait_for_service` 上，**连 "Filter is waiting for the map." 都打不出来** | `logs/p1diag_agent.log` 第 2854–2857 行 | 由 ① 连带解决；另加 `make_plan` 非致命处理（查不到可达性时保留该 frontier 而不是删除，更不是崩节点） |
| ③ | `always_send_full_costmap: false`：costmap_2d 只在启动时发一次完整栅格，之后只发 `costmap_updates` 增量。Filter 用 `gridValue()` 查任意格，必须拿到完整栅格；节点一旦 respawn 就永远等不到 | `Filter is waiting for the global costmap.` 连续刷 8 次 | 改 `true`（注释写清理由） |
| ④ | `base_footprint` **有两个父帧**：轮式插件发 `odom → base_footprint`，ORB-SLAM 发 `map → base_footprint`，tf2 在"两棵不相连的树"之间来回切 | `Connectivity Error looking up robot pose: Could not find a connection between 'map' and 'base_footprint' … Tf has two or more unconnected trees`（每轮 11 次） | `publishOdomTF: false`，机器人地图位姿只由估计器拥有 |

顺带修掉的三个真实 bug（都有 traceback / 离线复现为证）：

- `tf2.TransformException` 直接打死 Filter/`SimpleRobot`（`respawn` 后丢掉已收到的 costmap）→ 改非致命。
- `informationGain_NUMBA` 用显式 numba 签名 `nb.int64[:]`，而 rospy 把 `int8[]` 反序列化成
  `array.array('b')`，`np.array(...)` 得到 **int8** → `TypeError: No matching definition for argument
  type(s) … array(int8, 1d, C)`，每个质心每周期抛一次，节点必然崩。→ 每条地图消息转一次 int64。
  离线复现：`tools/selfcheck_loo.py` 第 1 项。
- `loo_observability.py::CameraModel.to_camera` 里用了未定义的 `t_cb`（应为 `self.t_cb`）→ 任何
  `strategy != native` 的臂只要开始打分就 `NameError`。离线自检抓到的。

### 2.2 打通后的原生闭环证据（`results/runs/p1loop5/`，native，240 sim-s）

| 证据 | 实测 |
|---|---|
| goal 真的下发了 | `goals.csv` **128 行 = 64 goal + 63 "Goal reached"**；第 2 个 goal 就是 frontier `(2.20, -0.30)` |
| `cmd_vel` 真的在发 | `cmd.csv` **2332 行**，`max │vx│=0.20 m/s`、`max │wz│=0.50 rad/s`（正是 TEB 的限幅） |
| 机器人真的在动 | `odom.csv` 1121 行，位移 **5.34 m**，路径 **8.85 m** |
| 地图真的在更新 | `coverage.csv`：`known_cells 5718`、`free_area 35.09 m²` |
| SLAM 真的在跟踪 | `slam.csv` 3601 位姿；离线对齐后 **ATE(RMSE)=0.145 m**、RPE=0.352 m |
| Filter 真的在输出 | `[fdbg]` 心跳 247 条 / 546 周期，每周期 6–7 个质心全部 KEPT（ig 0.7–2.3 > 阈值 0.5） |
| move_base 真的活着 | 该轮 `process has died` = 0、`Connectivity Error` = 0 |

**诚实保留的问题**：`state.csv` 里 `state=4`(OK) 与 `state=5`(LOST) 在 20–30 ms 内交替 6 次，
所以 `tracking_ok_pct` 只有 50%。但同轮 `[obs-instr]` 全程 `state=2`（ORB-SLAM 内部 OK）、
3601 个位姿、63 个目标到达 —— **跟踪实际是好的，是"发布出去的 state 在抖"**。未修，需单查。

### 2.3 统一动态像素掩码接口（一个开关，两臂共用）

- 入口：`Frame::ApplyDynamicMask`（`orb_slam2_ros`），在 `ExtractORB` 之后立即执行，
  **只**过滤关键点与其描述子行，保持 `mvKeys[i] ↔ mDescriptors.row(i)` 不变；
  不改匹配策略、不改 KF 插入、不改局部建图、不改 BA。
- 开关：`orb_slam2_ros/use_dynamic_mask`（launch 里 `use_dynamic_mask:=true|false`）。
  **关掉时连订阅都不创建**，前端与作者逐字节一致 —— 这是两臂公平性的机械保证。
- 掩码来源：`dynamic_mask.py`，**只用传感器数据**：深度 → 反投影 → 用*上一帧位姿*重投影 →
  自运动补偿后的深度残差 → 形态学开+膨胀 → 连通域尺寸筛选。不读 GT、不读 `model_states`、
  不向 `scenario_driver` 询问任何东西。
- 同一个节点顺便发布测量得到的动态物体（中心/尺寸/常速度），供 C++ 查询模块使用。

### 2.4 只读导出（已在一轮真跑里写成）

`results/runs/p1loop5/export/`：`map_points.csv` 287528 行（id、3D、`n_obs`）、
`keyframes.csv` 2744、`edges.csv` 94949、`poses.csv` 1008、`camera.json`（内参 + `T_base_camera`）、
`static_occupancy.npz`（栅格 + 原点 + 分辨率）、`manifest.json`。
全部来自已发布的 SLAM 话题，**零 C++ 改动**（最容易证明"只读"的实现方式）。

---

## 3 阶段二：三处建模口径修正 + 纯 C++ 视觉查询

### 3.1 `loo_occlusion.py`：$w_{\rm rel}$ 不再冒充遮挡概率

修正前：`1 - n_real/n_exp` 被直接当成"动态遮挡概率" `p_occ` 喂给规划器。这是**把测量说成了原因**：
该比值同时包含动态物体、运动模糊、弱纹理、前端门控，前端无法归因；而且经验可靠度与物理遮挡概率
不是同一个量（前者是"过去这个视角看到的比例"，后者是"未来被挡住的可能性"）。

修正后：`ReliabilityField` 输出**经验观测可靠度** $w_{\rm rel}\in[0,1]$，规划器取其**下置信界**
（不确定性必须抬高代价）：`strength_eff = strength · w_lo`。
类改名为 `ReliabilityField`（`OcclusionField` 保留为别名），`predict()` → `predict_reliability()`。
动态遮挡概率改由 C++ 查询模块的**三维动态物体常速度预测**给出 —— 两者语义彻底分开。

### 3.2 `loo_observability.py`：$H_z = S^\top H_\xi S$

修正前：`Hn = S^{-1} H S^{-1}`。若物理扰动 $\delta\xi = S z$（$S=\mathrm{diag}(\sigma_t,\sigma_t,\sigma_t,\sigma_r,\sigma_r,\sigma_r)$
是"要求精度"），链式法则给出 $H_z = S^\top H_\xi S$，而旧式相当于把归一化边缘标准差
**乘以** $s_i$，即"要求越松看起来越好"，方向反了。

修正后：`H_z = H ⊙ (s s^\top)`，$C_z=(H_z)^{-1}$ 的对角即 $(\sigma_{\xi,i}/s_i)^2$（无量纲）；
最弱方向用**平滑 soft-max**：$\mathrm{softmax}_\tau(\sigma_{z,i})=\tau\log\sum_i e^{\sigma_{z,i}/\tau}$
（恒 ≥ 真实 max，因而保守，且对 $H$ 是 $C^\infty$，避免特征值交叉处的折点被优化器当成跳变）。
另加 `softmax_frac` 参数与 `worst_marginal_std`/`lam_min_hard` 诊断量。

离线自检（`tools/selfcheck_loo.py`，13 项全过）中最关键的一项：

```
[PASS] strength scales with +1 power of s (H_z = S^T H S)
       base=12.903184 scaled=25.806369 ratio=2.000000 (want 2.0)
```

即把每个要求精度放大 2 倍，strength 正好放大 2 倍（旧式会缩小 2 倍）。

### 3.3 纯 C++ 局部视觉查询模块（`ws/src/visual_query`）

- 80–150 点下采样快照：`LocalVisualQuery::downsample`（按到传感器距离贪心 + 体素去重），
  自检里 4000 点 → 120 点。**不动 SLAM 前端特征预算**。
- 四维可微权重 $w_i(q,t)=w_{\rm fov}\cdot w_{\rm stat}\cdot w_{\rm dyn}(t)\cdot w_{\rm rel}$：
  视场边界/距离窗用 sigmoid 过渡（像素域 $\sigma$），静态遮挡用**三线性密度 + Beer–Lambert 透射率**
  （无硬射线），动态遮挡用椭球**近似符号距离 + 常速度预测**再套 sigmoid，$w_{\rm rel}$ 逐点给定。
- `query(pose,time)`：单视点归一化几何信息（$H_z=S^\top H S$ + 平滑最弱方向）；
  `query_pair(pose_a,t_a,pose_b,t_b)`：共视点数/权重、视差均值与**最小视差**、连续性
  （借鉴 APACE 的 `parallax_util` 思路）。
- 离线自检 27 项全过（`ws/src/visual_query/src/selfcheck_visual_query.cpp`），含：

```
[PASS] translation sweep has no jump (max 1mm step < 2%)   max step rel change=0.000558
[PASS] rotation sweep has no jump (max 0.1deg step < 2%)   max step rel change=0.002830
[PASS] fov gate never reaches exactly zero (no hard rays)
[PASS] lambda_g = 0 gives exactly zero information
[PASS] constant-velocity prediction clears the occlusion after 3 s
[PASS] dynamic gate is smooth in time (10 ms step < 10%)   max rel change=0.007151
[PASS] a static wall attenuates the static gate            w_stat_mean=0.678621
[PASS] identical poses -> zero parallax, full continuity
       query() = 11.6 us, query_pair() = 40.7 us (120-point snapshot)
```

即：**无零梯度平台、无硬射线、$w_{\rm rel}$ 语义正确、耗时远低于 100 ms 控制周期**。

---

## 4 阶段三：TEB 内部 g2o 两条有界软代价边

来源：`curl -L https://codeload.github.com/rst-tu-dortmund/teb_local_planner/tar.gz/refs/heads/noetic-devel`
→ `ws/src/teb_local_planner`（源码包，**不是** deb），在此之上新增：

| 边 | 类型 | 误差 | 说明 |
|---|---|---|---|
| `EdgeVisualObservability` | 一元（`VertexPose`） | $\sqrt{w}\,\max(0,1-g)$，$w=\lambda_g\cdot \Delta t_i$ | 按**该顶点的暴露时间**加权，惩罚视觉几何不足；有界（$\le\sqrt{w}$） |
| `EdgeCoVisibility` | 二元（两个 `VertexPose`） | $\sqrt{w}\,[\,(1-\text{continuity})+\max(0,1-\frac{\text{parallax}_{\min}}{\text{ref}})\,]$ | 惩罚跨帧共视中断与视差过小 |

实现要点（逐条对任务书）：

- **数值雅可比**：两条边都**不写** `linearizeOplus()`，直接吃 g2o 默认数值微分。
- **有界 + 不用视觉硬阈值拒绝前进轨迹**：两式的误差均被 clamp 到 $[0,\cdot]$ 且有上界。
- **联合优化原生位姿与 $\Delta t$**：边只挂在原生 `VertexPose` 上，$\Delta t$ 仍由原生
  `EdgeTimeOptimal`/速度/加速度边决定；视觉项只是**附加**代价。
- **外层刷新、内层优化**：`TebLocalPlannerROS::computeVelocityCommands` 里每周期
  `visual_query_->refresh()` 后 `optimizer_.setVisualQuery(...)`，内层 g2o 迭代只读。
- **固定感知时间间隔配对**：`AddEdgesCoVisibility` 按**累计时间**找第一个满足
  $\sum\Delta t \ge \Delta t_{\rm vis}=0.45$ s 的顶点 $j$，而不是"隔 m 个节点"，
  因此 TEB 自动增删节点不会改变共视口径。
- **开启多同伦类**：`enable_homotopy_class_planning: True` 保持；每条候选拓扑用同一个
  optimizer 与同一组边，视觉代价进入各拓扑的代价比较。
- **权重为 0 时完全不建边**：`weight_visual_observability==0` / `weight_covisibility==0` 直接 `return`，
  连查询对象都不创建 → 作者图逐字节复现（这就是"权重清零检验"）。

离线自检（`ws/src/teb_local_planner/src/selfcheck_visual_edges.cpp`，11 项全过）：

```
[PASS] lambda_g = 0 gives exactly zero cost          cost=0.000000
[PASS] lambda_c = 0 gives exactly zero cost          cost=0.000000
[PASS] cost is larger where the geometry is worse    far=0.570878 good=0.000000
[PASS] cost vanishes at the best pose                good=0.000000
[PASS] cost is bounded by sqrt(lambda_g * dt)        far=0.570878 bound=0.894427
[PASS] numerical Jacobian is finite                  dg/dx=-0.575592 dg/dtheta=-0.159071
[PASS] Jacobian is stable across eps=1e-4 and 1e-5   dx: -0.575592 vs -0.575592
[PASS] Jacobian points towards better geometry       dg/dx=-0.575592
[PASS] cost is continuous along a 2 m sweep          max jump=0.002679
[PASS] a pair that keeps its co-visibility costs less close=0.095163 far=0.811124
[PASS] the binary cost is bounded                    far=0.811124
```

---

## 5 阶段四.1：四项正确性自检

| 自检项 | 状态 | 证据 |
|---|---|---|
| 权重清零（$\lambda_g=\lambda_c=0$ 与原 TEB 一致） | ✅ 离线 + **在线**双重通过 | 离线 `[PASS] lambda_g = 0 gives exactly zero cost`；在线 B0 臂日志 `[visual_query] disabled (both visual weights are 0: author graph)` 且不打印边计数、TEB 耗时回到 73.6 ms |
| 自定义视觉边梯度方向与数值稳定性 | ✅ 离线通过 | `dg/dx=-0.575592` 指向更优几何；$\epsilon=10^{-4}$ 与 $10^{-5}$ 结果一致到 $10^{-6}$；2 m 扫掠无跳变 |
| 动态遮挡预测时序/坐标/外参一致性 | ⚠️ 部分 | 常速度预测的时序一致性在 C++ 自检里过了（3 s 后遮挡清除、10 ms 平滑）；**投影核对日志需要一轮带动态演员的真跑**，见 §6 |
| TEB 单次求解耗时 | ⚠️ 已插桩待跑 | `[teb-solve] n=… last=… mean=… max=… ms`（每周期一对 steady_clock），与 10 Hz 控制周期对比 |

### 5.1 构建状态与"deb 还是源码包"的证据（任务书 §2.2）

```
SUMMARY stage1=0 stage2=0                       # logs/build_final.log，两个阶段全绿、0 error
[run_task] teb source : SOURCE (workspace build) # rospack -> ws/src/teb_local_planner
[run_task] teb lib     : 49 deps resolved        # ldd 全部解析
rospack plugins --attrib=plugin nav_core | grep teb
  teb_local_planner .../ws/src/teb_local_planner/teb_local_planner_plugin.xml
ctypes.CDLL("libteb_local_planner.so")
  -> /home/wcqw/.../ws/devel/lib/libteb_local_planner.so
ldd ws/devel/lib/libteb_local_planner.so
  libvisual_query.so => /home/wcqw/.../ws/devel/lib/libvisual_query.so
```

真跑里的"插件确实是我这份源码包"的直接证据，是这些**只存在于新代码里的日志行**：

```
B0 臂（lambda=0）:  [visual_query] disabled (both visual weights are 0: author graph)
Ours 臂（lambda>0）: [visual_query] adapter subscribed to orb_slam2_rgbd/map_points,
                                     gridmapper/rectified_map, dynamic_objects
                     [visual_query] enabled (lambda_g=1, lambda_c=1, dt_vis=0.45 s)
```

即 **B0 臂在真跑里复现了作者图**（权重清零检验的在线版本），而 Ours 臂的查询确实挂上了。

### 5.2 实测 TEB 单次求解耗时（自检第 4 项）——**不达标，如实报告**

`[teb-solve]` 是每周期一对 `ros::WallTime` 的实测统计（不是估算）：

```
B0        : n=1059  mean=142.23 ms  max=573.898 ms
B1        : mean=129.22 ms
Ours      : mean=134.75 ms
Ours_no_cov: mean=125.79 ms
```

而 `controller_frequency: 10.0`（100 ms 周期）。move_base 自己在真跑里也报了
`Control loop missed its desired rate of 10.0000Hz`。

**结论**：**当前配置下 TEB 本身就跑不进 100 ms**，添加两条视觉边还要再慢 2–12 ms。
这是底座的既有问题（`enable_homotopy_class_planning: true` × `no_outer_iterations: 2`
× `no_inner_iterations: 3` × 2 个同伦类），不是视觉边引入的；但**它直接决定了本轮 A/B
不可解释的部分**：控制周期从 100 ms 掉到 ~140 ms，四臂的轨迹差异里混入了"采样时刻不同"
的成分。下一步必须先把这个解决掉（降 `max_number_classes` / 迭代数，或换
`optimization_algorithm_levenberg` 的稀疏求解设置），再谈视觉边带来的差异。
**未解决，如实记录。**

---

## 6 阶段四.2：4 臂 A/B（固定起终点、动态横穿遮挡场景）

### 6.1 协议

- 场景：`scenario:=visual`（两块**无碰撞体**的视觉遮挡板分别横穿
  `x=1.6, y∈[-2.2,2.4]` 与 `y=1.2, x∈[-2.0,2.6]`；无碰撞 → 激光看不见 → 隔离"视觉遮挡"通道）。
- 目标序列：`results/fixed_goals_ab.csv`，**相对 agent 启动时刻**的 12/45/80/115 s，
  四个航点全部在 p1loop5 中原生探索里**验证过可达**；
  四臂完全一致（`goto_goal.py` 单一下发器，`with_decision_maker:=false`）。
- 感知底座一致：同一个 `dynamic_mask.py` 掩码（`use_dynamic_mask:=true`）、同一份 costmap、
  同一份记录器；四臂**只差 TEB 的两个权重**。
- 每臂 1 次（240/230 sim-s）。**任务书要求每臂 ≥3 次重复，本轮因共享重型锁被另一条线长期占用，
  只做到 1 次**——这一点直接影响结论强度，见 6.3。

### 6.2 结果表（最终一轮：四臂全部成功到达）

同一 `git_sha=d5d8156`、同一场景 `visual`、同一固定起终点（`results/fixed_goals_ab.csv`，
航点 `(2.20,-0.30)`、t=20 s，直线段跨过 `visual_panel_a` 的 `x=1.6` 走廊）、同一掩码、
同一 costmap。每行由 `tools/eval_run.py` 从该次九条 CSV 机械算出；GT 只在此离线使用。

| arm | 到达 | cmd_vel | 路径 m | 净位移 m | 覆盖率 m² | ATE m | 失跟 s | 跟踪 OK % | **TEB 均值 ms** | 视觉边数 |
|---|---|---|---|---|---|---|---|---|---|---|
| `B0` 原生 TEB | ✅ 1/1 | 106 | 2.067 | 2.063 | 8.58 | 0.049 | 0.0 | 100.0 | **74.7** | 0 / 0（按设计不建边） |
| `B1` +静态几何/共视 | ✅ 1/1 | 71 | 2.041 | 2.035 | 6.49 | 0.024 | 0.0 | 100.0 | **135.6** | 6 / 2 |
| `Ours` = B1+动态预测 | ✅ 1/1 | 62 | 2.061 | 2.057 | 5.97 | 0.028 | 0.0 | 100.0 | **172.0** | 15 / 7 |
| `Ours_no_cov` 去共视边 | ✅ 1/1 | 81 | 2.038 | 2.037 | 9.09 | 0.033 | 0.0 | 100.0 | **126.4** | 7 / 0 |

每条臂日志里的结构证据（`graph_edges` 含原生边，故远大于视觉边数）：

```
B1          : vertices= 6 visual_edges=  6 covis_edges= 2
Ours        : vertices=15 visual_edges= 15 covis_edges= 7
Ours_no_cov : vertices= 7 visual_edges=  7 covis_edges= 0
B0          : [visual_query] disabled (both visual weights are 0: author graph)
```

### 6.3 结论（保守，只写数据支持的）

1. **管线完全打通，且在真跑里可验证**：权重清零 → 作者图（B0 + 11 项离线自检）；
   权重 >0 → 边真的进 g2o（上面的计数）；四臂在固定起终点上**全部到达**、全程
   `state==2`、ATE 0.024–0.049 m。**加视觉代价没有破坏任务完成能力**（这是本轮最硬的正面结论）。
2. **实时性：仍然不达标，但已经从"灾难"降到"超预算"**。同一轮对比：
   `74.7 ms (B0) → 126.4 ms (仅一元边) → 135.6 ms (一元+二元) → 172.0 ms (含动态预测)`，
   而控制周期是 100 ms。也就是说：原生 TEB 已经用掉 75% 的周期余量，两条视觉边再吃掉 50–97 ms。
   **10 Hz 在任何一臂都没达到**，move_base 真跑里也会报 `Control loop missed its desired rate`。
3. **优化前后的代价对比是可复现的证据链**（同一个"共视边"项）：
   - 未优化（第 4 轮）：`Ours` 臂 TEB **1857.7 ms**、单次最坏 **28.7 s**、400 顶点、
     230 sim-s 里只发出 77 条 `cmd_vel`；
   - 优化后（本轮）：`Ours` 臂 **172.0 ms**、15 顶点、62 条 `cmd_vel`、按时到达。
   三步改动分别是：① 限制 `max_samples=30`（切断"挪不动→图变大→更慢"的正反馈）；
   ② 关掉查询里的静态遮挡透射率项（与 costmap 重复，且是每次数值雅可比的第二大开销）；
   ③ `query_pair` 去掉两个只为诊断存在的内部单点查询（40.7→11.3 µs），并把共视边改为
   每隔一个顶点配对（`visual_covis_stride=2`，仍是固定感知间隔）。
4. **"视觉代价让轨迹更优"——本轮数据不支持**：四臂路径长度差 <2%（2.038–2.067 m），
   覆盖率 5.97–9.09 m²，ATE 0.024–0.049 m，**n=1 且差异都在噪声量级**。
   λ=1 时这两条边对轨迹是"小扰动"，这与 TEB 原生的权重量级（`weight_obstacle=50`、
   `weight_optimaltime=1`、`weight_kinematics_nh=1000`）是一致的——**下一步必须先做 λ 标定**，
   再谈方法优劣。
5. **下一步（按收益排序，均未做）**：
   a. 两条边的**解析雅可比**（任务书允许初版用数值微分，实时化必须换）；
   b. λ 标定（在独立 seed 上标定，再跑评测 seed），使视觉项与原生态在可比量级；
   c. 只对关键顶点加一元边、共视边只挂跨遮挡窗口的配对；
   d. 完成后跑 4 臂 × ≥3 次重复，并对"预测遮挡量 vs 实际可用观测"做逐帧相关性检验。

### 6.4 本轮 A/B 的诚实边界

- **每臂只有 1 次重复**（任务书要求 ≥3）：共享重型锁被另一条线连续占用，单次排队最长 2400 s
  超时；期间按 BRIEF 不抢、不 kill。
- 本轮固定起终点只有**一个航点**（`(2.20,-0.30)`）：前几轮用过的 `(5.27,0.87)`、
  `(0.10,0.10)`、`(-1.8,1.1)`、`(0.3,-2.7)`、`(2.20,-4.30)` 在多个臂里都出现
  `move_base status=4 "Failed to find a valid plan"`（**目标点选在不可达区域**，
  不是规划器结果）。`(2.20,-0.30)` 是唯一"每次都到达"的航点，其直线段跨过视觉遮挡板走廊，
  因此仍是一个合法的"固定起终点 + 动态横穿"对照，但**轨迹很短（2 m）**，能提供的区分度有限。
- `ATE/RPE` 在不同路径长度之间不可直接比较；本轮四臂路径几乎相同，所以这一轮的数字可比，
  但 n=1 仍不足以支撑显著性。
- `tracking_ok_pct`/`lost_s` 受"published state 在 4/5 之间抖动"影响（见 §2.2），
  本轮的 100% 只说明"没有彻底失跟"。

### 6.5 相关性日志：预测遮挡 vs 实际跟踪内点（任务书 4.3 要求）

`tools/correlation_mask_n.py` 把 `dynamic_mask.py` 逐帧预测的遮挡像素数与 ORB-SLAM 前端
`[obs-instr]` 的真实可用观测数按**仿真时间**（而不是按索引）配对，只取 `state==2`（跟踪中）的帧：

```
masked px vs N            : pearson=+0.019 spearman=-0.214
masked px vs valid_depth  : pearson=-0.445 spearman=-0.515      <-- 预期的负相关
bin (masked px quartile) -> mean usable depth keypoints
  q4  masked_px  14340..70216   mean valid_depth_kp=417.6   （最低的一档）
```

即：**预测遮挡越多的帧，前端可用的深度观测显著越少**（Spearman −0.52），方向正确；
而对原始 `N`（关键点数）相关性弱（+0.02 / −0.21），因为 `N` 被 `ORBextractor` 的 1000 特征预算
截顶，遮挡更多表现为"可用深度少"而不是"关键点少"。逐帧数据：`export/occlusion_vs_inliers.csv`。

### 6.4 失败与未跑（保留）

| 项 | 说明 |
|---|---|
| 第一轮 A/B（`results/ab_table_round1_badroute.csv`） | 固定路线里含 `(2.20,-4.30)`，四臂最后一条目标都是 `Failed to find a valid plan`。**这是目标点选错，不是规划器结果**，该轮整表作废但保留 |
| 第二轮的 B1 那一格 | 我在其运行期间手工执行了 `cleanup.sh`（为修孤儿进程），把该 arm 的节点杀了 → `teb_norm` 为空。已在 STATUS §02:30 记录 |
| 每臂重复次数 | **1 次**（要求 ≥3）：重型锁被另一条线（DynoSAM）连续占用，单次排队最长 2400 s 超时 |

---

## 7 复现

```bash
source /home/wcqw/explorb_line_20261121/env.sh
source /home/wcqw/explorb_line_20261121/ws/devel/setup.bash
# 注意：上面第二行会覆盖 ROS_PACKAGE_PATH / CMAKE_PREFIX_PATH，本地前缀要重新前置
export ROS_PACKAGE_PATH=/home/wcqw/explorb_line_20261121/localroot/opt/ros/noetic/share:$ROS_PACKAGE_PATH

./build.sh                       # 两个变量都在 setup 之后重新前置（见脚本注释）

# 离线自检（不需要 Gazebo / master）
python3 tools/selfcheck_loo.py
# C++ 查询模块
g++ -O2 -std=c++14 -I ws/src/visual_query/include -I/usr/include/eigen3 \
    ws/src/visual_query/src/local_visual_query.cpp \
    ws/src/visual_query/src/selfcheck_visual_query.cpp -o /tmp/vq && /tmp/vq
# TEB 两条边
g++ -O2 -std=c++14 -I ws/src/teb_local_planner/include -I/usr/include/eigen3 \
    -I /opt/ros/noetic/include \
    ws/src/teb_local_planner/src/selfcheck_visual_edges.cpp -o /tmp/se \
    -L/opt/ros/noetic/lib -lg2o_core -lg2o_stuff -lroscpp -lrosconsole -lrostime \
    -lrosconsole_log4cxx -lrosconsole_backend_interface -lcpp_common -lroslib && /tmp/se

# 一次原生探索（重型锁内）
heavy --line A --note run --max-wait 1200 -- bash run_task.sh native static task1 240 0

# 4 臂 A/B（固定起终点路线，每条 240 sim-s）
bash tools/run_ab.sh static 240 1
```
