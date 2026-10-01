# Cubli 项目真实状态审计（2026-09-25）

## 审计结论先行

当前工作区**没有**三飞轮点平衡闭环（G3C）模型，也没有行走模型。磁盘中存在 G3B 的三飞轮点支撑试验模型、Roll/Pitch 角度测量、以及独立的 `cubli_point_allocator` 函数；它们尚未被连接为闭环控制器。

此前把“G3C 正在接线/已推进”说成实际项目进度是不准确的。本次审计以 `outputs/cubli_clean` 中当前源文件和一次实际 MATLAB 命令执行为唯一依据；不以聊天叙述补全任何实现。

## 后续开发附记（2026-09-26）

审计完成后，G3I 已得到两轮一致的用户实测结果，并据此新增了 `build_cubli_clean_point_balance.m` 与 `point_balance` 入口。G3C 现已具有三路执行器、Roll/Pitch 角度与角速度、2 ms 力矩保持和控制分配的**源码实现**；尚无成功仿真的验收结果。因此本审计中“G3C 尚未实现”的历史结论应更新为：**已实现但未验证**。详细运行步骤见 `CUBLI_G3C_TEST_PROTOCOL.md`。

同日发现原始顶点姿态将质心错误放在世界 Y 轴，现已修正为 `[0,0,0.129903810568] m`。旧 G3I 矩阵已废弃；在修正几何上重新取得两轮完全一致的 ZERO-baseline 脉冲数据，并已写入参数文件。G3C 仍为“已实现但未验证”，下一项验收是新的 2 s 闭环运行。

随后新的 G3C 早期状态记录显示角速度持续增加且三路力矩饱和。根因进一步定位为飞轮组件质心不平衡：旧轮心坐标为 `(+.065,0,0)`, `(0,-.065,0)`, `(0,0,+.065)` m，三只等质量飞轮的总质心不在方块中心。轮心已改为等质量质心平衡布局，ZERO 基线用户实测接近机器精度，X/Y/Z 新辨识数据已写入参数文件。

在新布局下，G3C 2 s 末端状态实测 `qf=[-6.96649e-9, 5.57322e-9] rad`、`qdf=[1.09754e-7,-8.78042e-8] rad/s`，力矩峰值 `[0.000832607,0.0022619,0.00142929] N*m`，数值上通过 2 s 点平衡。用户报告延长到 10 s 后动画仍保持点平衡，但尚未提供 10 s 全程峰值日志；参数默认时长现改为 10 s，最终长时数值验收待回报。

最终 10 s 日志已返回：`duration=10.000 s`、`qPeak=[0.000872885,0.000698308] rad`、`qdPeak=[0.00468416,0.00374734] rad/s`、`tauPeak=[0.000832607,0.0022619,0.00142929] N*m`。结合用户报告的稳定 Mechanics Explorer 动画，G3C 理想顶点支撑点平衡 10 s 验收通过。下一开发阶段转为 G2D：自由接触到棱支撑的捕获与稳定保持。

G2D 首个隔离模型已新增：`build_cubli_clean_edge_capture.m` 调用可参数化的 G2C 物理模型构建器，使用质心平衡后的飞轮位置独立生成 5 s 自由接触捕获模型，并输出 `g2d_*` 日志。MATLAB 无界面静态检查本次未能在 45 s 内完成，因此模型尚未标记为编译或仿真通过；需要用户本机运行 `mode="edge_capture"` 验证。

## 接触基线与 G2D 崩溃附记（2026-09-26）

本日诊断 G2D 的 `degenerate mass distribution` 报错，牵出三项更正：

1. **G2A 的"通过"没有证据。** 该构建器当时没有 `Transform Sensor`、没有 `To Workspace`，`run_cubli_clean` 直接丢弃其输出（`%#ok<NASGU>`）。所谓通过只是"仿真跑完了"。实测一个应当静止的方块以 31.7 °/s 持续自转（2 ms 下 45.7 °/s、COM 漂移 0.914 m）。G2A 现已重建为带 5 路日志和数值门槛的门槛，并带证伪对照，见 `CUBLI_G2A_TEST_PROTOCOL.md`。

2. **G2D 崩溃与时长无关。** 模型每次都在 t≈9.3–9.4 s 崩（二分实测 9.3 过 / 9.4 败）。直接原因是 `build_cubli_clean_face_to_edge.m` 的轮速限幅条件符号写反，限幅器从未生效。修法已单跑验证但尚未改入源码。

3. **G2C 的既有数字作废。** 接触族的机构局部求解器已从 ODE1 改为 ODE3（同为 1 ms）。旧的 `x: -0.0005 到 0.1813`、`z: 0.0740 到 0.1073` 是 ODE1 的数字，不会复现；已重测为 `x: -0.00011 到 0.11735`、`z: 0.07460 到 0.10571`。

同日本审计方法本身的一处更正：先前"机构局部求解器没有隐式选项"的说法**过宽**。只有 `MultibodyLocalSolverChoice`（`ODE1..ODE8`，显式）没有隐式选项；`Solver Configuration` 上另一组 `UseLocalSolver`/`LocalSolverChoice` 是 Simscape 网络求解器，**有** `NE_BACKWARD_EULER_ADVANCER`、`NE_TRAPEZOIDAL_ADVANCER` 等隐式选项。当时把 `'BackwardEuler'` 赋给了前者而被拒，从未实测后者。

该实验已补跑：在 ODE1 @ 1 ms 的 G2A 上分别开启 `UseLocalSolver` 并置为 `NE_TRAPEZOIDAL_ADVANCER` 与 `NE_BACKWARD_EULER_ADVANCER`（均 @1 ms），净偏航**四位数完全不变**（302.5397° / 302.5397° / 302.5397°，关时为 302.5397°）。因此该网络求解器在此模型上是 **no-op**（机构自成分区，网络无非机构内容），这是实测结论而非推断。刚性罚函数接触的阶数问题只能靠 `MultibodyLocalSolverChoice` 解决。

## 审计范围、方法与证据等级

- 工作区：`C:\Users\20722\Documents\Codex\2026-09-24\ban\outputs\cubli_clean`
- MATLAB：`E:\MATLAB\R2024b\bin\matlab.exe`
- 检查方法：文件清单、源代码全文/关键词检索，以及无界面 MATLAB 执行。
- `.slx` 是由构建脚本生成的结果；本审计以对应 `.m` 构建脚本为准。`*.autosave` 不视作实现证据。
- “用户已验证”表示此前由用户在本机 Mechanics Explorer 中实际观察或贴出数值；本次没有把它伪装成本次自动化验证。

状态术语：

| 标记 | 含义 |
|---|---|
| 已实现且可运行 | 有实际源文件，且本次自动运行通过，或已有明确的用户运行结果；会注明来源。 |
| 已实现但未验证 | 源码存在，但本次没有成功完成相应模型的运行。 |
| 只存在接口/占位代码 | 只有函数、参数或预留接口，没有接入可执行功能链。 |
| 尚未实现 | 没有可执行源文件、入口或模型构建代码。 |

## 所需项目项逐项审计

| 项目项 | 状态 | 磁盘证据与关键函数 | 审计说明 |
|---|---|---|---|
| 三只飞轮模型 | **已实现但未在本次完整仿真验证** | `build_cubli_clean_point_geometry.m`，`localWheel`；G3B 分支的 `Wheel X/Y/Z Joint`、`Wheel X/Y/Z Rotor` | G3B 确实创建三只 Revolute Joint、三只 Rotor，并把三个力矩输入端设为 `InputTorque`。G3A 只放置固定的可视飞轮，不是可驱动飞轮。|
| Roll/Pitch 状态 | **已实现但仅测角、未形成完整状态** | `build_cubli_clean_point_probe.m`：`g3_roll`、`g3_pitch` | 两个理想支撑转动副启用 `SensePosition` 并记录为 rad。没有启用 `SenseVelocity`，因此没有 `rollRate/pitchRate`。|
| 双轴测量 | **已实现但未在本次完整仿真验证** | `build_cubli_clean_point_probe.m`：`Roll Angle`、`Pitch Angle`、`Log Roll`、`Log Pitch` | 双轴角度测量存在。它不是惯性姿态传感器；读数来自理想顶点支撑中的两个关节。|
| `cubli_point_allocator` | **已实现且独立函数可运行** | `cubli_point_allocator.m`，函数 `cubli_point_allocator(q,qd,P)` | 该函数执行二维 PD、2×3 阻尼伪逆分配和三路限幅。2026-09-25 已实际在 MATLAB 无界面运行，见“实际测试记录”。|
| 三飞轮闭环 | **尚未实现** | 全目录检索仅命中 `cubli_point_allocator.m` 本体、参数文件和历史文档；`run_cubli_clean.m` 无 `point_balance`/`G3C` 分支 | 没有 MATLAB Function 块调用 allocator；没有三路 `tau_X/tau_Y/tau_Z` 控制输出；没有速度测量；没有闭环模型构建器。|
| 控制器接入（点平衡） | **尚未实现** | `build_cubli_clean_point_probe.m` 只含 `X/Y/Z Probe Command` 常量 | 当前 G3B 明确将 X、Y 常量设为 0，Z 常量设为 0.01 N·m。这是方向探针，不是控制器。|

## 实际文件地图

| 功能 | 实际文件 | 关键函数/模块 | 当前真实作用 |
|---|---|---|---|
| 统一入口 | `run_cubli_clean.m` | `mode` 分支 | 当前默认 `"point_probe"`；可选 G0、G1、G2A/B/C、G3A/B；没有 G3C/G4。|
| 参数 | `cubli_clean_parameters.m` | `cubli_clean_parameters` | 单一参数源。`P.control.point.*` 存在，但参数存在不等于控制器已接入。|
| 分配器 | `cubli_point_allocator.m` | `cubli_point_allocator` | 独立函数，不被任何 builder 或入口调用。|
| G3 顶点机构 | `build_cubli_clean_point_geometry.m` | `build_cubli_clean_point_geometry(probeMode)`、`localWheel` | `probeMode=true` 时生成 G3B 三电机飞轮机构；顶点是两个理想 Revolute Joint，不是物理地面点接触。|
| G3 方向探针 | `build_cubli_clean_point_probe.m` | `build_cubli_clean_point_probe` | 给三路均留有输入，但现在仅 Z 路施加 +0.01 N·m；记录 Roll/Pitch。|
| G1 棱平衡 | `build_cubli_clean_edge_rig.m` | `localSetController`、`Edge Controller` | 单轴理想棱支撑、一个飞轮、完整 PD 关节闭环；与点平衡独立。|
| G2C 面到棱 | `build_cubli_clean_face_to_edge.m` | `localSetStandupController`、`Face-to-Edge Controller` | 自由 6-DOF + 接触 + 一个 Y 飞轮的短时过渡试验；不是长期平衡。|
| G2A/G2B | `build_cubli_clean_contact_settle.m`、`build_cubli_clean_wheel_spin_test.m` | 对应 build 函数 | 分别是接触静置与固定台架单飞轮测试。|
| 行走 | 无 | 无 | 当前没有 G4 builder、状态机、步态或入口。|

## 点平衡链路的真实断点

现有 G3B 的信号链实际为：

```text
三个常量探针命令 ──> 三个 Simulink-PS 转换器 ──> X/Y/Z 飞轮关节
两个关节位置 ──> PS-Simulink 转换器 ──> g3_roll / g3_pitch 记录
```

缺失的链路为：

```text
Roll/Pitch 角度与角速度 ──> 离散控制器 ──> cubli_point_allocator
     └────────────────────────────────────> tau_X / tau_Y / tau_Z ──> 三个飞轮
```

所以“看见只有 Wheel Z 在转”符合当前代码：`Z Probe Command` 是 `0.01`，而 `X Probe Command` 与 `Y Probe Command` 都是 `0`。这不是三飞轮闭环失效，而是三飞轮闭环根本尚未写入模型。

## 实际测试记录

### A. 分配器：本次已执行并通过

实际执行的 MATLAB 命令为：

```matlab
P = cubli_clean_parameters();
cubli_point_allocator([0;0], [0;0], P)
cubli_point_allocator([0.01;0], [0;0], P)
cubli_point_allocator([0;0.01], [0;0], P)
```

实际输出：

```text
[0;0]            -> [0; 0; 0]
[0.01;0]         -> 1e-3 * [-0.5878; 0.6033; 0.7447]
[0;0.01]         -> 1e-3 * [0.1400; -0.1622; 0.0643]
```

这证明函数在 MATLAB 中可解析、参数字段完整、输出为三元素力矩向量。它**不**证明闭环稳定，也不证明响应矩阵足够可靠。

### B. G3B 现有 `.slx` 快照：本次仿真未完成

本次对现有 `Cubli_Clean_G3B_PointProbe.slx` 执行了无界面 `load_system` + `sim`，模型 StopTime 为 0.10 s。该命令在 30 s 后仍未产生日志或退出；两个由本次审计启动的 MATLAB 进程已被停止，用户原有的交互式 MATLAB 进程未触碰。因此，**G3B 不能在本次审计中标记为“已运行通过”**。该结果只说明当前无界面运行未完成，不能单独判断其原因是模型、Simscape 初始化、许可证/图形环境，还是外部 MATLAB 资源竞争。

本次也没有把历史口头叙述重新标记为自动通过。已有的、可追溯的用户运行结果如下：

| 模式 | 既有运行观察 | 当前审计判定 |
|---|---|---|
| G0 | 用户确认灰色地面、蓝方块、三只橙色飞轮位置已合理 | 用户已验证几何显示；本次未重跑。|
| G1 | 用户给出 10 s 末端量约为零，且最大指令力矩 0.0148 N·m | 用户已验证单轴理想棱平衡。|
| G2A / G2B | 用户确认接触/台架基础测试可运行 | 用户已验证基础试验。|
| G2C | 用户观察到面到棱滚动；延长到 5 s 曾报 `Free Body has a degenerate mass distribution on its follower side` | 短时过渡有证据；长期运行**失败且未修复**。|
| G3A | 用户确认顶点支撑几何，但地面仍有视觉相交 | 理想顶点机构有证据；没有物理点接触。|
| G3B | 用户给出角度探针输出；观察到不规则运动且主要只有 Z 飞轮转 | 探针数据存在；不是闭环验证。|

## 对响应矩阵的风险说明

`P.control.point.responseMatrix` 的数值来自先前 0.10 s 探针与零力矩基线的末端差值。顶点平衡是重力不稳定系统，0.10 s 内的自然发散会混入电机响应；因此该矩阵只足够说明三个轮子耦合方向不同，**尚不足以作为 10 s 稳定控制器的可信标定**。

在连接闭环前，必须先获得可重复、短脉冲、同初值的响应数据，并记录角度和角速度。否则将错误地把重力发散当成飞轮控制效应，调参容易表现为“无规则运动”。

## 历史描述更正

以下历史描述与磁盘事实不一致，现正式更正：

1. “G3C 三飞轮闭环接线正在完成/已推进”——不成立。当前没有 `build_cubli_clean_point_balance.m`、G3C `.slx`、闭环信号线或 G3C 入口。
2. “Roll/Pitch 状态已经接成完整闭环状态”——不成立。只有位置，未读取角速度。
3. “`cubli_point_allocator` 已接入控制器”——不成立。它是可运行的独立函数，未被 Simulink 图调用。
4. “G2C 的 2 ms 延迟已解决长期退化”——没有证据。用户已报告 5 s 时仍出现退化错误。

旧的 `CUBLI_PROJECT_STATUS.md` 是历史进度文档，包含上述过度表述；应以本审计文档为当前管理基线。

## 当前真实进度

```text
G0  三维几何显示                 ── 有用户验证
G1  理想棱支撑单轴闭环           ── 有用户验证
G2  自由接触/单轮过渡试验        ── 仅短时；长期接触退化未解决
G3A 理想顶点支撑几何             ── 有用户验证
G3B 三飞轮开环方向探针           ── 源码存在，非稳定控制
G3C 三飞轮点平衡闭环             ── 未实现
G4  多步行走                     ── 未实现
```

## 下一步唯一合理的开发任务

**先建立 G3C 的可重复双轴小信号辨识试验，不直接接入点平衡控制器。**

该任务应从 G3B 独立新建，验收内容固定为：

1. 三轮各自可选择短脉冲，其他两轮严格为零；
2. Roll、Pitch、Roll rate、Pitch rate 全部记录；
3. 每条试验从同一很小的初始倾角开始，并有相同初始条件的零输入基线；
4. 使用远短于重力发散时间的脉冲/观察窗口，计算每轮对四个状态量的可重复增量；
5. 重复三次，先检查符号和方差，再更新 `responseMatrix`。

完成该项并给出实际日志后，下一项才是新建 G3C 闭环 builder：2 ms 状态读取、三路 allocator 输出、三路执行器、饱和与日志。这样每一步都有可证伪的验收结果，而不是在不可靠标定上直接叠加控制器。
