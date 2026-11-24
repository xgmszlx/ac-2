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
**P5：4 臂 × 3 次重复全部跑完，12/12 episode 全部到达目标。**
- **主判据 RPE 上看不出方法效应**（0.0403–0.0443，跨度与臂内波动同量级）；
  ATE 上 `B0 0.0456 → Ours 0.0167` 的差**不能独立成立**（四臂观测到的栅格面积差 19%，
  轨迹不同构），已降级为待验证假设；
- **动态遮挡项在本轮根本没被触发**：269 个对齐样本里只有 3 帧掩码非零 →
  `Ours` 与 `B1` 物理上不该有差别，实测也确实没有（见 §6.5）；
- **跟踪丢失时长 = 0.00 s**（用 SLAM 内部状态算；发布层的 state 抖动已从该项剔除，§6.6）；
- **实时性仍未达标**：`74.2 → 127.6 / 144.9 / 155.2 ms`，10 Hz 周期在任何一臂都没满足；
- 代价已从最初的 **1857 ms（单次最坏 28.7 s）** 降到 155 ms（三步优化见 §6.3）；
- 结论强度：n=3、单一场景、单一 2 m 航点、动态事件未触发 —— **这是一个诚实的"未测出效应"**，
  下一步的首要改动是换一条真正穿过遮挡板走廊的 episode（§6.3 第 7 条）。

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

### 5.2 实测 TEB 单次求解耗时（自检第 4 项）——**仍不达标，如实报告**

`[teb-solve]` 是每周期一对 `ros::WallTime` 的实测统计（不是估算）。
最终一轮（4 臂 × 3 次重复，`results/ab_table.csv`）：

```
B0         : mean= 74.2 ±  2.7 ms      <- 原生 TEB 就用掉 74% 的 100 ms 周期
Ours_no_cov: mean=127.6 ±  2.7 ms      <- 只加一元边
B1         : mean=144.9 ±  8.0 ms      <- 一元 + 共视二元边
Ours       : mean=155.2 ± 18.7 ms      <- 再加动态遮挡预测
```

`controller_frequency: 10.0`（100 ms）。真跑里 move_base 也会报
`Control loop missed its desired rate of 10.0000Hz`。

**结论**：**10 Hz 在任何一臂都没满足**；原生 TEB 已经占掉 74%，两条视觉边再加 53–81 ms。
更严重的一次（未优化时）是 `Ours` 达 **1857.7 ms、单次最坏 28.7 s** ——
那一步的代价结构与三步优化见 §6.3/§6.4。**未解决，如实记录。**

---

## 6 阶段四.2：4 臂 A/B（固定起终点、动态横穿遮挡场景）

### 6.0.1 前置门禁：A/B 数必须来自源码 TEB（不是 deb）

这条门禁是本报告所有 §6 数字的**前提**，并且已经脚本化、可重跑：

```bash
tools/verify_source_teb.sh <task_id> <lambda_g> <lambda_c>     # 退出码 0 才算通过
```

三项判定（实测全部命中）：

```
===== sample 3 pid=1391115 =====
cmdline: /opt/ros/noetic/lib/move_base/move_base cmd_vel:=/cmd_vel ...
/home/wcqw/explorb_line_20261121/ws/devel/lib/libteb_local_planner.so        <-- 进程实际加载的库
[visual-constraint] ... vertices=15 visual_edges=15 covis_edges=7 graph_edges=97 <-- 边进了 g2o
[teb-solve] n=67 last=96.5 ms mean=149.6 ms max=378.4 ms                        <-- 只有新代码才打这行
[gate] PASS   maps_lines=2 source_path_hits=2 visual_edges=6 covis_edges=2 teb_solve_lines=2
```

符号对比（deb vs 源码）：

```
ws/devel/lib/libteb_local_planner.so              EdgeVisualObservability|EdgeCoVisibility = 45
localroot/.../libteb_local_planner.so.deb-orig    EdgeVisualObservability|EdgeCoVisibility =  0
```

**已完成的 12 次 A/B 全部满足这条门禁**：12/12 日志都有 `[teb-solve]`；λ>0 的三臂
`visual_edges = 6 / 15 / 7`；B0（λ=0）打印
`[visual_query] disabled (both visual weights are 0: author graph)` 且没有任何边计数
——这就是"权重清零"自检的**实机版**（离线版是 `[PASS] lambda_g = 0 gives exactly zero cost`）。
`tools/run_ab.sh` 现在把这条门禁做成**起跑前断言**：验证不过就直接 `exit 1`，不产出任何 A/B 数字。

> 背景：本地前缀里也有一份 deb 的 `teb_local_planner`，早期版本会被它 shadow 掉，
> 表现为"四臂完全一样"的假阴性。修复分两层：`run_task.sh` 把 `ws/src` 与 `ws/devel/lib`
> 前置；编排窗口另外把 `localroot` 里那个 `.so` 换成了指向源码构建的符号链接（原库备份为
> `.deb-orig`），两条路都指向源码库。

### 6.1 协议

- 场景：`scenario:=visual`（两块**无碰撞体**的视觉遮挡板分别横穿
  `x=1.6, y∈[-2.2,2.4]` 与 `y=1.2, x∈[-2.0,2.6]`；无碰撞 → 激光看不见 → 隔离"视觉遮挡"通道）。
- 目标序列：`results/fixed_goals_ab.csv`，**相对 agent 启动时刻**的 `t=20 s`、单一航点 `(2.20,-0.30)`；
  从 spawn 到该点的直线段跨过 `visual_panel_a` 的 `x=1.6` 走廊，即"动态横穿遮挡"。
  四臂完全一致（`goto_goal.py` 单一下发器，`with_decision_maker:=false`）。
- **路线定稿过程（provenance，必须保留）**：本节的单航点路线是**主窗口（orchestrator）于 03:53/03:58
  两次修订后定稿的，不是本线的原始设计**。修订依据与两次假设的措辞写在 `results/fixed_goals_ab.csv`
  文件头；判据是**实测成功率**（把磁盘上所有臂、所有代的 goal 结果全数，86 次 dispatch）：
  `(2.2,-0.3)` 35/35=100%、`(4.2,-0.5)` 3/3、`(0.1,0.1)` 9/15=60%、`(5.27,0.87)` 0/19=0%、
  `(3.5,0.0)` 0/1。原四航点路线保留为 `results/fixed_goals_ab.4wp.orig.csv`。
- 感知底座一致：同一个 `dynamic_mask.py` 掩码（`use_dynamic_mask:=true`）、同一份 costmap、
  同一份记录器；四臂**只差 TEB 的两个权重**。
- **每臂 3 次重复**（`budget=180–230` sim-s，12 次运行全部保留在 `results/ab_table.csv`）。
  早前四航点路线的各代（`ab_table_round1_badroute.csv` … `round4_slow.csv`）**整表作废但保留**，
  原因见 6.6。

### 6.0.2 ⚠️ 一次真实的记录覆盖事故（已修，并在此留痕）

**事实**：第 1–5 轮我重复使用了同样的 `task_id`（`ab_<arm>_visual_r<rep>`），
后一轮的 `run_task.sh` 会写同一个 `results/runs/<task_id>/` 目录，
于是**前一轮的同名运行目录与其日志被覆盖**；同时我为了存档把 `ab_table.csv`
用 `mv` 移走，导致主窗口 02:25 读到的表只剩 2 行。
**这违反了任务书"保留全部失败运行""记录每次运行的 git SHA 与参数快照"的硬要求。**

**现在改成了（三处硬约束）**：

1. **task_id 唯一**：`results/runs/<task>_<confighash>/`，`confighash =
   sha1(git_sha, dirty, scenario, budget, seed, 掩码开关, 决策层开关, 路线文件, 权重参数)[:7]`；
   目录里已有 `clock.csv`/`metrics.json` 时**直接 FATAL 退出**，绝不覆盖。
2. **`results/ab_table.csv` 只追加**：不再 `mv`、不再截断；写入前先读全表，
   同一 `task_id` 已存在时**拒写并警告**。
3. **表格由脚本从磁盘生成**：`tools/ab_summary.py` 扫描所有
   `results/runs/*/metrics.json`，产出 `results/ab_table_generated.md` 与 `.csv`
   —— 本报告 §6.2 的数字就是从这两个文件里取的，**不是手抄**。

**已不可恢复的部分（如实列出）**：第 1–5 轮的同名运行目录与本轮之前的 `logs/ab_*_*.log`。
**仍然可复核的部分**：每轮当时的 `metrics.json` 快照被保留成了
`results/ab_table_round1_badroute.csv`、`round2_partial`、`round3_zeroedges`、
`round4_slow`、`round5_slow_binary`（含 `git_sha` 列），以及 `results/lambda_sweep.csv`。
**最后一次完整轮（4 臂 × 3 次重复，12 个目录）全部在盘上**，是本报告结论的唯一依据。

### 6.2 结果表（4 臂 × **3 次重复**，12/12 全部到达）

同一场景 `visual`、同一固定起终点（`results/fixed_goals_ab.csv`：航点 `(2.20,-0.30)`、t=20 s，
直线段跨过 `visual_panel_a` 的 x=1.6 走廊）、同一掩码、同一 costmap；
四臂只差 TEB 的两个权重。每行由 `tools/eval_run.py` 从该次九条 CSV 机械算出
（GT 只在此离线使用）。均值±标准差来自 `tools/ab_summary.py --agg`。

| arm | n | 到达 | **RPE m**（主判据） | ATE m（辅） | 路径 m | 覆盖率 m² | 失跟 s（内部状态） | cmd_vel | **TEB 均值 ms** |
|---|---|---|---|---|---|---|---|---|---|
| `B0` 原生 TEB | 3 | 3/3 ✅ | 0.0403 ± 0.0006 | 0.0456 ± 0.0055 | 2.056 ± 0.016 | 8.27 ± 0.92 | **0.00** | 104.7 ± 1.5 | **74.2 ± 2.7** |
| `B1` +静态几何/共视 | 3 | 3/3 ✅ | 0.0443 ± 0.0008 | 0.0219 ± 0.0043 | 2.038 ± 0.005 | 7.04 ± 1.39 | **0.00** | 71.0 ± 0.0 | **144.9 ± 8.0** |
| `Ours` = B1+动态预测 | 3 | 3/3 ✅ | 0.0434 ± 0.0030 | 0.0167 ± 0.0112 | 2.029 ± 0.039 | 6.73 ± 1.52 | **0.00** | 65.3 ± 3.5 | **155.2 ± 18.7** |
| `Ours_no_cov` 去共视边 | 3 | 3/3 ✅ | 0.0419 ± 0.0018 | 0.0298 ± 0.0040 | 2.040 ± 0.006 | 8.82 ± 0.26 | **0.00** | 83.3 ± 2.5 | **127.6 ± 2.7** |

**这张表怎么读（口径声明，2026-11-25 06:00 修订）**

- **RPE 是主判据、ATE 只是辅**：四臂 RPE 差异（0.0403–0.0443，跨度 10%）**小于或接近臂内波动**
  （±0.0006–0.0030），即 **RPE 上看不出任何方法效应**。
- **ATE 的差异（B0 0.0456 vs Ours 0.0167）不能单独当结论**：虽然四臂路径长度只差 1.3%
  （2.029–2.056 m），但**覆盖的栅格面积不同**（B0 8.27 m² vs Ours 6.73 m²，−19%），
  也就是"看到的场景不一样"，ATE 对"哪些路段被观测到"很敏感。
  → **ATE 上的 2–3 倍差是待验证假设，不是结果**；要定论必须做"同一时间窗/同一区段"的分段 ATE。
- **本实现里的 RPE 也只是弱漂移代理**（1 s 窗相对位移的离散度），对 2 m 直行段几乎只反映曲率；
  换更严格的 RPE（如 TUM 的 `rpe_trans` 分段对齐）后数字可能不同，**未做**。
- **失跟时长用 SLAM 自己的状态**（`[obs-instr] state==2`）算：四臂都是 **0.00 s**，
  99.6% 的采样为 OK。发布出来的 `/orb_slam2_*/info/state` 在 4/5 之间抖动，是**发布层现象**，
  已从"跟踪丢失时长"里剔除（见 §6.6 口径说明）。

> 下表的机器生成版本：**`results/ab_table_generated.md` / `.csv`**（由
> `python3 tools/ab_summary.py --md --agg` 从 12 个 `metrics.json` 生成）。
> 手写表格仅作阅读用，任何复核请以生成文件为准。

逐次明细（`results/ab_table_generated.csv`，12 行；原始行亦在 `results/ab_table.csv`）：

| task | ATE m | 路径 m | 覆盖 m² | TEB ms |
|---|---|---|---|---|
| ab_B0_visual_r1/r2/r3 | 0.0493 / 0.0393 / 0.0483 | 2.067 / 2.038 / 2.063 | 8.58 / 7.24 / 9.00 | 74.7 / 76.7 / 71.3 |
| ab_B1_visual_r1/r2/r3 | 0.0236 / 0.0170 / 0.0251 | 2.041 / 2.032 / 2.042 | 6.49 / 8.62 / 6.02 | 135.6 / 150.1 / 148.9 |
| ab_Ours_visual_r1/r2/r3 | 0.0283 / 0.0060 / 0.0158 | 2.061 / 1.985 / 2.040 | 5.97 / 5.74 / 8.48 | 172.0 / 158.5 / 135.0 |
| ab_Ours_no_cov_visual_r1/r2/r3 | 0.0335 / 0.0304 / 0.0255 | 2.038 / 2.034 / 2.046 | 9.09 / 8.57 / 8.81 | 126.4 / 130.7 / 125.7 |

每条臂日志里的结构证据（边确实进图）：

```
B1          : vertices= 6 visual_edges=  6 covis_edges= 2
Ours        : vertices=15 visual_edges= 15 covis_edges= 7
Ours_no_cov : vertices= 7 visual_edges=  7 covis_edges= 0
B0          : [visual_query] disabled (both visual weights are 0: author graph)
```

### 6.3 结论（只写数据支持的）

1. **管线正确、且每一步都可核对**：权重清零 → 作者图（B0 真跑 + 11 项离线自检）；
   权重 >0 → 边进入 g2o（上面的计数）；**12/12 episode 全部到达目标**、全程 `state==2`、
   零失跟。加视觉代价**没有**破坏任务完成能力。
2. **RPE（主判据）上看不出方法效应**：`B0 0.0403±0.0006` / `B1 0.0443±0.0008` /
   `Ours 0.0434±0.0030` / `Ours_no_cov 0.0419±0.0018`，跨度 10%，与臂内波动同量级。
   **ATE 上 B0 0.0456 → Ours 0.0167 的差（约 2–3 倍标准差）不能独立成立**：
   四臂虽然路径长度只差 1.3%，但**观测到的栅格面积差 19%**（8.27 vs 6.73 m²），
   ATE 对"哪些路段被看到"高度敏感。→ 这一项**降级为待验证假设**，需要分段（同一时间窗/同一区段）
   ATE 才能判。
3. **动态遮挡项在本轮实际上没有被触发**：`corr_ours` 的 269 个对齐样本里只有 **3 帧**掩码非零，
   "预测遮挡 vs 实际跟踪内点"的相关性测不出（pearson +0.09）。这同时解释了为什么
   `Ours` 与 `B1` 的 RPE/路径几乎相同——**没有遮挡事件，动态项无从起作用**。
   长路线上的方向性证据（Spearman −0.52）保留在 §6.5 作为"机制在事件发生时可测"的旁证。
4. **实时性仍不达标**：`B0 74.2 ms` → `B1 144.9` → `Ours 155.2` → `Ours_no_cov 127.6`，
   控制周期 100 ms。原生 TEB 已用掉 74% 的余量，两条视觉边再吃掉 53–81 ms。
   真跑里 move_base 也会报 `Control loop missed its desired rate`。
5. **覆盖率不是本方法的目标，且数据支持"它确实让机器人更保守"**：
   `B0 8.27 m²` vs `Ours 6.73 m²`（−19%）、`Ours_no_cov 8.82 m²`。
   即视觉代价会牺牲一点探索收益换取约束质量——这正是任务书里要的"有界代价"的预期行为，
   但也说明**权重需要标定**（§6.4b）。
6. **代价优化是有效的、可复现的**：同一项共视边，未优化时 `Ours` 曾达 **1857.7 ms**
   （单次最坏 28.7 s、400 顶点、230 sim-s 只发 77 条 `cmd_vel`），三步优化后 **155.2 ms**
   （15 顶点、65 条 `cmd_vel`、按时到达）。三步：`max_samples=30`、查询侧静态遮挡项默认关、
   `query_pair` 去掉诊断用的内部单点查询（40.7→11.3 µs）+ 共视边 `stride=2`。
7. **下一步（按收益排序，均未做）**：
   a. **换一条真正穿过遮挡板走廊、且停留足够长的 episode**（本轮最大缺陷：动态项没被触发）；
   b. 解析雅可比（实时化前提）；λ 标定（§6.4b 是起点）；
   c. 分段/同窗 ATE + 严格 RPE 重算，再谈精度结论；
   d. 独立 seed 上 ≥5 次重复并做显著性检验。

### 6.4b λ 敏感性标定（`results/lambda_sweep.csv`，同一固定起终点、同一 seed）

任务书要求在独立设置上先标定权重。这里用同一固定 episode、只改
`lambda_g = lambda_c = λ`（**这是标定，不是评测轮**）：

| λ | 到达 | 路径 m | 净位移 m | 覆盖率 m² | ATE m | TEB ms | cmd_vel | 视觉边（一元/二元） |
|---|---|---|---|---|---|---|---|---|
| 1（A/B 的 `Ours`） | 1/1 | 2.061 | 2.057 | 5.97 | 0.028 | 172.0 | 62 | 15 / 7 |
| 10 | 1/1 | 2.029 | 2.013 | 7.36 | 0.0166 | 199.2 | 62 | 7 / 3 |
| 40 | 1/1 | **2.393** | **1.953** | 7.98 | 0.0314 | 190.6 | 86 | 6 / 2 |

读数（保守）：

- **λ 从 1 提到 40 时，路径开始变"绕"**：路径 +18%、净位移 −5%（2.057→1.953 m），
  即视觉代价开始真正影响轨迹形状——这是本方法唯一一个"方向正确"的迹象；
- 但 **n=1/λ**，且 ATE（0.0166 / 0.028 / 0.0314）的差异处于噪声量级，**不能**读成精度提升；
- TEB 耗时与 λ 关系不大（172 / 199 / 191 ms），说明耗时主要取决于**顶点数与边数**，
  不是权重值——也就是说"标定 λ"不会进一步恶化实时性。

### 6.5 相关性日志：预测遮挡 vs **实际跟踪内点**（任务书 4.3 要求）

两条日志流已经足够，不需要为它单独加物理量。新增 `tools/join_mechanism.py`（离线，不需要锁/不重跑）
把三方按下表对齐，产出 `results/runs/<task>/mechanism.csv` + `mechanism_summary.json`：

| 预测侧（TEB/掩码） | 实测侧（SLAM） |
|---|---|
| `dynamic_objects.csv` 的 `n_mask_px`（就是 $w_{dyn}$ 的输入：被跟踪动态物体的遮挡像素） | `[track-instr] matched=`（跟踪后仍关联的地图点数，外点与失效点已排除） |
| `[visual-constraint] … visual_edges= covis_edges= vertices=` | `[obs-instr] N= valid_depth_kp= state= masked_out=` |

列：`t, w_dyn_proxy, n_blobs, covis_edges, visual_edges, vertices, matched_inliers,
N_keypoints, valid_depth_kp, state, masked_out`（按仿真时间最近邻 ±0.2 s 对齐）。

**实测（合格 run，四臂同一协议）**

| run | masked_px vs matched_inliers | masked_px vs valid_depth_kp | masked_px vs N_keypoints |
|---|---|---|---|
| `ab_Ours_visual_r4_43417b7` | r=+0.107, ρ=+0.258 (n=119) | r=+0.355, ρ=+0.344 | r=−0.102, ρ=−0.268 |
| `ab_B1_visual_r4_412f1c9` | r=+0.069, ρ=+0.183 (n=119) | r=+0.267, ρ=+0.272 | r=+0.030, ρ=+0.011 |
| `corr_ours`（同协议） | r=+0.090, ρ=+0.083 | r=+0.228, ρ=+0.243 | r=−0.034, ρ=−0.141 |

**怎么读（这是本轮最重要的负结论）**

- **方向不成立**：预测遮挡与"实际跟踪内点"之间**没有负相关**（r≈+0.07~+0.11，符号还偏正）。
  分箱曲线也不单调：`q1..q4` 的 matched inliers 是 178 → 280 → 292 → 374（越高越"多"），
  因为后半程地图点更多、遮挡帧与"地图变丰富"在时间上是混在一起的——**这是一个混淆**，
  而不是方法效应。
- **原因**：整段 230 s 里掩码非零的帧只有个位数（`corr_ours` 是 3/269），
  预测侧几乎没有方差；`q1..q3` 的 `masked_px` 全为 0。**这条 2 m 航线上动态遮挡事件根本没被触发**，
  所以既测不出相关性，也解释得通"`Ours` 与 `B1` 的 RPE/路径一致"。
- **有意义的那部分**：在最早的 6 航点长路线上（`masked px vs valid_depth_kp`，
  616/1129 帧掩码非零）得到 **r=−0.445, ρ=−0.515**，方向正确。说明"机制在事件发生时可测"，
  但要真正检验动态项，必须换一条**机器人确实穿过遮挡板走廊、并在遮挡期间停留**的 episode。
- `covis_edges vs matched_inliers` 无法计算（n≈0~1）：`[visual-constraint]` 是 5 s 节流打印，
  与 2 s 一次的 track 行几乎不重叠。**下一版应把 covis 计数写进每周期日志**（已在 §6.8 列为未完成项）。

### 6.6 失跟时长的口径（`lost_s` 曾是假指标，已更换）

| 口径 | 含义 | 本轮取值 |
|---|---|---|
| ~~`state.csv` 的 `state==4`~~ | 这是 `/orb_slam2_*/info/state` **发布层**的值，在本平台 4/5 之间 20–30 ms 抖动 | ~~四臂 50–53% OK / 失跟 85–113 s~~ **弃用** |
| **`[obs-instr]` 的 `state==2`** | ORB-SLAM `Tracking::eTrackingState` 的**内部**状态（OK=2） | **四臂全部 `lost_internal_s = 0.00`、`track_ok_internal_pct = 99.6%`** |

`tools/eval_run.py` 现在同时输出 `lost_internal_s`/`track_ok_internal_pct`（内部状态，权威）
与 `published_state_lost_s`/`published_state_ok_pct`（发布层，仅作现象记录，
`metrics.json` 里带 `lost_s_note` 说明），**任何方法间结论只能用前者**。

### 6.7 每次运行的 git SHA + 参数快照（任务书 4.3）

`run_task.sh` 现在在起跑**之前**写 `results/runs/<task>/meta.json`，内容包括：
`git_sha`、`git_branch`、`teb_package` + `.so` 的 mtime/size、`teb_library_is_workspace_source`、
`lambda_g`、`lambda_c`、`visual_dt`、`visual_use_dynamic`、`use_dynamic_mask`、
`with_decision_maker`、`with_fixed_goals` + 路线文件、`scenario`、`seed`、`ground_truth_online: false`。
样例（`results/runs/corr_ours/meta.json`）：

```json
{ "task": "corr_ours", "strategy": "native", "scenario": "visual", "seed": 0,
  "git_sha": "567bc85fc07f4a35288313aaeed3fd11151057f8",
  "git_branch": "research/explorb-locobs-20261124",
  "teb_package": ".../ws/src/teb_local_planner",
  "teb_library": "2026-11-25 04:00:00 +0800 16846024",
  "teb_library_is_workspace_source": true,
  "lambda_g": 1.0, "lambda_c": 1.0, "visual_dt": 0.45, "visual_use_dynamic": true,
  "use_dynamic_mask": true, "with_decision_maker": false, "with_fixed_goals": true,
  "ground_truth_online": false }
```

（本轮 12 次 A/B 是在加这个字段**之前**跑的，它们的 meta 由 `results/ab_table.csv`
的 `git_sha`/`git_branch` 列 + 各臂日志里的 `[visual-constraint]` 行共同构成等价快照；
从下一次运行起每个 episode 都自带 `meta.json`。）

### 6.7b 可完全追溯的一轮（唯一 id + 每 run `meta.json` 之后）

上面 §6.2 的 12 次运行早于"唯一 id + meta.json"机制，**目录在盘上、`git_sha` 在 `metrics.json` 与
`ab_table_round*.csv` 里，但没有 `meta.json`**。为补上这一条，最后又跑了一轮**四臂 × 1 次**，
全部使用新机制（`results/runs/ab_<arm>_visual_r4_<confighash>/`，每目录带 `meta.json`）：

| run | 到达 | 路径 m | 覆盖 m² | RPE m | ATE m | 失跟(内部) s | TEB ms |
|---|---|---|---|---|---|---|---|
| `ab_B0_visual_r4_911b20b` | **0/0**（失败，见 §6.8） | – | – | – | – | 0.00 | – |
| `ab_B1_visual_r4_412f1c9` | 1/1 ✅ | 2.035 | 4.79 | 0.0409 | 0.0201 | 0.00 | 148.3 |
| `ab_Ours_visual_r4_43417b7` | 1/1 ✅ | 2.026 | 5.24 | 0.0426 | 0.0072 | 0.00 | 158.7 |
| `ab_Ours_no_cov_visual_r4_49bfe7b` | 1/1 ✅ | 2.024 | 4.84 | 0.0821 | 0.0239 | 0.00 | 125.8 |

（机器生成版本：`results/ab_table_generated.md`；失败行 `goals=0` 保留在上表但不进均值。）

### 6.8 失败与未跑（保留）

| 项 | 说明 |
|---|---|
| 第一轮 A/B（`results/ab_table_round1_badroute.csv`） | 固定路线里含 `(2.20,-4.30)`，四臂最后一条目标都是 `Failed to find a valid plan`。**这是目标点选错，不是规划器结果**，该轮整表作废但保留 |
| 第二轮的 B1 那一格 | 我在其运行期间手工执行了 `cleanup.sh`（为修孤儿进程），把该 arm 的节点杀了 → `teb_norm` 为空。已在 STATUS §02:30 记录 |
| 每臂重复次数 | 主 3 次（§6.2），补跑 1 次（§6.7b）；**未做到每臂 3 次 × 两轮** |
| **`ab_B0_visual_r4_911b20b`（失败，保留）** | 我在该 episode 运行期间手工执行了新加保护的 `cleanup.sh`——**保护是在那次运行开始之后才加的，pid 文件还不存在，于是它把这一轮杀掉了**（`n_goal=0`、无路径）。这是操作失误，不是算法失败；之后 `cleanup.sh` 已能正确拒绝（实测 `cleanup: REFUSED - run_task.sh (pid …) is running an episode.`） |
| 第 1–5 轮运行目录被同名覆盖 | 见 §6.0.2；已修复（唯一 id），历史数据以 CSV 快照保留 |

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

---

## 8 附录：为什么可以相信"跑的是源码 TEB"（三重自证）

这类实验最容易被质疑的一点是"你改了源码但运行时加载的是 deb"。本线用三条互相独立的机械证据
堵住这个口子，并且都在**每次运行的日志里**，不需要事后回忆：

1. **包解析层**（`run_task.sh` 启动时打印，并写入 `meta.json`）：
   `rospack find teb_local_planner` → `<ws>/src/teb_local_planner`；`[run_task] teb source : SOURCE`。
2. **符号/链接层**：`nm -C ws/devel/lib/libteb_local_planner.so | grep -cE 'EdgeVisualObservability|EdgeCoVisibility'`
   = **45**，同一 grep 对 `localroot/…/libteb_local_planner.so.deb-orig` = **0**；
   `ldd` 里 `libvisual_query.so => <ws>/devel/lib/libvisual_query.so`（`meta.json.teb_library_ldd`）。
3. **运行进程层**：`tools/verify_source_teb.sh` 在 episode 运行中读
   `/proc/<move_base pid>/maps`，必须命中 `<ws>/devel/lib/libteb_local_planner.so`，
   否则脚本 `exit 1`、A/B 不许开跑（`tools/run_ab.sh` 已把这条做成起跑前断言）。
   实测一次：`cmdline: /opt/ros/noetic/lib/move_base/move_base …` +
   `…/ws/devel/lib/libteb_local_planner.so`，`[gate] PASS`。

**外加每臂自述**：`[visual_query] enabled (lambda_g=…, lambda_c=…, dt_vis=…)` /
`[visual_query] disabled (both visual weights are 0: author graph)`——这四个字符串**只存在于源码构建**，
deb 版一个都没有；所以"B0 臂 = 作者图"和"B1/Ours 臂 = 带边"这两件事，都是运行时可验证的，而不是靠参数推断。

## 9 未完成项（明确列出，供下一轮接手）

| # | 未完成 | 卡在哪 / 下一步 |
|---|---|---|
| 1 | 动态遮挡项**从未被真正触发** | 需要一条"机器人穿过 `visual_panel_a`(x=1.6) 或 `visual_panel_b`(y=1.2) 走廊并停留"的 episode；本轮 230 s 内掩码非零仅个别帧 |
| 2 | `covis` 计数与 track 行的时间分辨率不匹配 | 把每周期 `w_dyn` 均值与 `covis_n` 写进 `[teb-solve]`/`[visual-constraint]` 行（一行改动），再重跑即可做相关 |
| 3 | 实时性不达标（74→126~172 ms vs 100 ms 周期） | 解析雅可比 → 稀疏加边 → 限 `max_samples`/同伦类数 |
| 4 | 严格 RPE（TUM `rpe_trans` 分段对齐）与分段 ATE | 本轮 RPE 是弱代理；分段 ATE 才能处理"四臂观测面积不同"的混淆 |
| 5 | 每臂重复次数 | 最后一轮做到 3 次；随后加入 meta.json 机制的一轮只跑到 1 次/臂（含 1 次被我误杀，见 §6.8） |
| 6 | 独立 seed 的评测轮 | 目前全部 seed=0（同一起点/同一演员相位） |
