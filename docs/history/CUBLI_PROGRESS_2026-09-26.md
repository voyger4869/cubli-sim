# Cubli 进度说明（2026-09-26）

> 本文是给项目负责人看的工作说明，记录本日对 G2D 报错的诊断、发现的缺陷、已做的修改与当前进度。
> 长期结论仍应回写 `CUBLI_PROJECT_STATUS.md` 与各门槛协议。

## 一句话结论

G2D 在 10 s 报的 `degenerate mass distribution` **不是"把时间设成 10"造成的**——模型每次都在 **t≈9.3–9.4 s** 崩，设 5 s 只是跑不到那一刻。沿这条线查下去，发现**三个互相独立的缺陷**，其中两个属于已经标记为"通过"的 G2A/G2C 接触基线。目前正在按"先修 G2A 接触基线"的路线推进，Stage 0 已完成并验证，Stage 1 基本完成。

## 一、发现的三个缺陷

### 缺陷 1：G2D 轮速限幅器符号写反（G2D 崩溃的直接原因）

`build_cubli_clean_face_to_edge.m` 的限幅条件：

```matlab
% 注释声称：Positive motor torque produces negative measured wheel speed ...
if abs(w)>=wmax && tau*w<0
  tau=0;
end
```

实测符号约定与注释**恰好相反**：

| t (s) | τ (N·m) | ω_wheel (rad/s) |
|---|---|---|
| 0.2 | +0.6362 | +693.1 |
| 5.0 | +0.1482 | +2658.6 |
| 8.0 | +0.2234 | +9213.7 |

正力矩**增大**正轮速 ⇒ 该拦的是 `tau*w > 0`，代码拦的是 `tau*w < 0`。**限幅器从未生效**，`P.motor.maxSpeed = 1800` 形同虚设，轮速单调发散。

**验证**：把条件改成 `tau*w>0` 后，同一个 10 s 运行**完整通过**，轮速被限制在 `[-182, +1828]`。因果链闭合。

**二分实测**（新会话、干净重建）：StopTime 9.3 通过、9.4 失败；新会话直接跑 10 s 同样失败。零力矩变体跑 10 s 正常 ⇒ 崩溃是被控动力学驱动出来的。

**状态：已定位、已复现、已验证修法，但尚未改入源码**（按路线排在 G2A 之后）。

### 缺陷 2：G2A 接触基线从未真正验证过

G2A 构建器没有 `Transform Sensor`、没有 `To Workspace`，`run_cubli_clean.m` 直接丢弃其输出。所谓"通过"的依据只是**仿真跑完了**，没有任何状态日志。

零力矩、方块平放、本应静止不动，实测（G2A 本体，10 s）：

| 局部求解器 | 净偏航 | COM xy 漂移 | 穿透 |
|---|---|---|---|
| ODE1 @ 2 ms（G2A 原设置） | **478.18°** | **1.84 m** | 9.47e-3 m |
| ODE1 @ 1 ms | **302.54°** | 8.54e-4 m | 7.04e-4 m |

原设置下方块已经在满场翻滚了。**状态：已修复（G2A 现已可观测且有数值门槛）。**

### 缺陷 3：自转的主因是积分器阶数，不是接触律

`MultibodyLocalSolverChoice = ODE1`（显式欧拉）配刚性罚函数接触。对照实验（零力矩、2 s，在 G2C 机构上）：

| 局部求解器 | 步长 | 虚假偏航 | COM 漂移 |
|---|---|---|---|
| ODE1 | 1 ms | 31.667 °/s | 1.06e-3 m |
| ODE1 | 0.5 ms | 13.273 °/s | 3.17e-5 m |
| ODE1 | 0.1 ms | 3.819 °/s | 5.61e-6 m |
| **ODE3** | **1 ms** | **-0.046 °/s** | 1.02e-3 m |
| ODE5 | 1 ms | 0.108 °/s | 1.42e-3 m |
| ODE8 | 1 ms | 1.002 °/s | 1.20e-2 m |

接触刚度（1e3–1e6）、阻尼（0–160）、初始穿透归零，结果全在 24–72 °/s 同一量级；删掉 Ground Contact 则为 0。**换 ODE3 降约 690 倍。**

约束（已实测）：`MultibodyLocalSolverChoice` 只提供 `ODE1|ODE2|ODE3|ODE4|ODE5|ODE8`，机构求解器**没有隐式选项**。
更正一处我先前的过宽表述：`Solver Configuration` 上还有另一组 `UseLocalSolver`/`LocalSolverChoice`（Simscape 网络求解器），**有** `NE_BACKWARD_EULER_ADVANCER` 等隐式选项。我当时把 `'BackwardEuler'` 赋给了前者而被拒，从未测过后者。预期在此模型上是 no-op，但尚未实测，不应写成结论。

## 二、已做的修改

### Stage 0 — 让参数文件说真话（已完成并验证）

| 文件 | 修改 |
|---|---|
| `cubli_clean_parameters.m` | `normalDamping` 55 → **160**（G2C/G2D 一直硬编码 160，G2A 读 55，两条接触门槛的接触律本来就不一致） |
| | 新增 `P.cube.assemblyMass`（原先在三个 builder 里各推导一次） |
| | 新增两档命名求解器档位 `P.sim.localSolver.contact/ideal` |
| | 新增 `P.contact.accept.*` 验收阈值 |
| `build_cubli_clean_face_to_edge.m` | 硬编码 `'160'` → 读参数；局部求解器改按档名取用；`totalMass` 改读 `assemblyMass` |
| `build_cubli_clean_contact_settle.m` | 同上；新增可选覆盖参数以便复现坏配置 |
| `build_cubli_clean_wheel_spin_test.m` | 局部求解器改按 ideal 档取用 |
| `build_cubli_clean_point_geometry.m` | 同上（G3A/B/C/I 共用） |

**验证证据**（重建后回读 `.slx` 参数，与改动前逐项比对）：

- G2B、G3A：**完全一致**，无污染
- G2C：**完全一致**（ODE1 @ 0.001，damping 160）
- G2A：两处**有意**变更 —— 局部步长 0.002 → 0.001、damping 55 → 160（统一接触族）
- G3C/G3I/G3B/G1/G0：未重建，代码路径未变

### Stage 1 — 让 G2A 可观测（基本完成）

- `build_cubli_clean_contact_settle.m` 新增 3 个独立 Transform Sensor（`SenseXYZ` / `SenseR` / `SenseOmegaZ`）+ 接触块 `SensePenetrationDepth` / `SenseFrictionalForceMagnitude`，共 5 路日志
- 外层求解器对齐 G2C/G2D：`MaxStep 0.001`、`RelTol 1e-4`
- 新增 `cubli_contact_report.m`（仿 `cubli_point_balance_report.m` 结构）
- `run_cubli_clean.m`：`contact_settle` 分支改为 assign 日志 + `g2a_report`；顺手删掉文件末尾遗留的活代码（`tEval`/`format long g`，它在**每个**模式下都会执行）

**两个刻意的设计决定：**

1. **判据不叫 `pass`，叫 `numericQuiet`。** G2A 在零输入零扰动下只测数值安静度，抓不到错误的摩擦系数、惯量、符号或过期几何。叫 `pass` 会被读成"接触模型是对的"。
2. **不用 `SenseAngle` 算速率。** 它无符号且截断在 [0,180]，实测会绕回（零力矩跑里 126.3°→170.0°→106.3°→74.6°，方块其实单向匀速转）。端点差在 10 s 窗口会绕回，36.73 °/s 恰好绕满 360° 时报 0.0 °/s。改用 `SenseOmegaZ`（有符号、无绕回）。

**端口名是探测出来的，不是猜的**：实测 `Spatial Contact Force` 的 sense 按对话框顺序追加为 `RConn2`、`RConn3`…，builder 里对此有断言，多加一个 sense 会显式报错而不是静默接错线。

## 三、进度推进情况

| 阶段 | 内容 | 状态 |
|---|---|---|
| 诊断 | 定位三个缺陷、复现崩溃、二分到 t≈9.35 s | **完成** |
| Stage 0 | 参数文件说真话，零行为污染 | **完成并验证** |
| Stage 1 | G2A 可观测 + 证伪对照 | **完成** |
| Stage 2 | 物理 vs 数值：删飞轮重测 | **完成，假设被否** |
| Stage 3 | 接触族启用 ODE3，重跑 G2A/G2C/G2D | **完成** |
| Stage 4 | 文档回写 | **完成** |
| 缺陷 1 | 限幅器符号修复 | **已验证修法，未改入源码（超出本次范围）** |

**进度确实推进了**：G2D 的报错从"不明原因"变成"已定位、已复现、修法已单跑验证"；G2A 从"无任何日志的假通过"变成"有 5 路实测日志、数值门槛和证伪对照的门槛"，且**已通过 10 s**。

## 四、证伪对照结果

证伪对照是必须的：一个从未在已知坏配置上失败过的报告不算证据。实跑（G2A 本体，10 s，零力矩）：

| 局部求解器 | **净偏航** | 峰值\|ωz\| | COM xy 漂移 | 穿透 | 判定 |
|---|---|---|---|---|---|
| ODE1 @ 2 ms（原设置） | **478.18°** | 811.15 °/s | 1.84 m | 9.47e-3 m | **FALSE**（4 项） |
| ODE1 @ 1 ms | **302.54°** | 45.79 °/s | 8.54e-4 m | 7.04e-4 m | **FALSE**（2 项） |
| **ODE3 @ 1 ms（当前）** | **0.268°** | 13.62 °/s | 1.13e-3 m | 4.30e-4 m | **true** |

量级分离极干净：净偏航 478° / 303° / **0.268°**，相差三个数量级。

**一处判据修正（过程中发现）**：最初我用 `mean|ωz|` 作速率判据，结果 ODE3 下净偏航只剩 0.268°（方块实际没转）却仍读 3.30 °/s——原因是 ω 在零点附近高频抖动而非漂移，`mean|ω|` 把零均值抖动当成了漂移。已改为主判据用**净积分偏航**（`|∫ω_z dt|`），它同时避开了 `SenseAngle` 的绕回问题和抖动敏感；峰值角速度与 COM 漂移降级为**粗差探测器**，并在参数文件里写明它们不区分 ODE1/ODE3。

**另需注意**：穿透判据余量偏紧——ODE3 实测 4.30e-4 m，限值 5.44e-4 m（1.27×）。方块按构造初始就压入 1.0× 静压缩，所以地板是 1.0×，1.5× 只允许 50% 瞬态超调。它确实能分离（ODE1@1ms 为 1.94×），但属临界，已在协议中写明。

## 五、Stage 2：`I_xy` 假设被否

原假设：三只飞轮全在 z=0 中平面，`Σm·x·y = 1.5e-4` 而 `I_xx = I_yy`，横向主轴恰在 ±45°，可能把摇摆暂态耦合成偏航。实验：删掉三只飞轮实体、按 0.50 kg 重算 `z0`，得到 `I_xy = 0` 的完美对称体。

| 求解器 | 含飞轮 | 裸方块 |
|---|---|---|
| ODE1 @ 1 ms | 302.54° | **60.68°** |
| ODE3 @ 1 ms | 0.268° | 0.649° |

**假设被否**：删掉飞轮只把 ODE1 的偏航从 302° 降到 61°，没有塌陷；而**一个完美对称的刚体不可能物理地产生偏航**，所以驱动量是积分器，`I_xy` 只是约 5 倍的放大器，ODE3 会把它抵消。

**直接后果：无需改动 `P.geometry.wheelCenters`，G2D 与 G3C 已验收的证据不受影响。** 这条实验很值得做——如果没做，会误判成几何问题，而改轮位会同时作废 G2D 和 G3C 的验收。

保留一条给 G4 的记录：`I_xy ≠ 0` 意味着行走中每次落地都可能产生真实的偏航偏置，这是架构输入。

## 六、Stage 3/4 结果

- 接触族（G2A/G2C/G2D）机构局部求解器：`ODE1` → **`ODE3` @ 1 ms**
- 理想族（G2B/G3A/B/C/I）保持 `ODE1` @ 2 ms **不变**（回读 `.slx` 参数逐项确认）
- G1/G0 未启用机构局部求解器，未触碰
- **G2C 数字已重测并作废旧数字**：旧 `x: -0.0005 到 0.1813`、`z: 0.0740 到 0.1073`（ODE1）→ 新 **`x: -0.00011 到 0.11735`、`z: 0.07460 到 0.10571`**（ODE3）。定性结论不变：z 达 0.1057 m、棱高 0.1061 m，确实发生了面到棱的抬升。
- 新文档：[`CUBLI_G2A_TEST_PROTOCOL.md`](CUBLI_G2A_TEST_PROTOCOL.md)；已更新 `CUBLI_PROJECT_STATUS.md`、`CUBLI_G2D_TEST_PROTOCOL.md`、`CUBLI_AUDIT_2026-09-25.md`

补跑了一个原本只是推断的实验：`Solver Configuration` 上另一组 `UseLocalSolver`/`LocalSolverChoice`（Simscape 网络求解器，**有**隐式选项）在本模型上是 **no-op** —— 开启并设为 `NE_TRAPEZOIDAL_ADVANCER` 或 `NE_BACKWARD_EULER_ADVANCER` 后净偏航四位数完全不变（302.5397°）。所以刚性接触的阶数问题只能靠 `MultibodyLocalSolverChoice` 解决，这条现在是实测结论而非推断。

## 七、下一步

1. **修缺陷 1 的限幅器符号**（`build_cubli_clean_face_to_edge.m` 的 `tau*w<0` → `tau*w>0`），单独一步，记录 10 s 结果
2. **G2D 控制器重做**：当前的 COM x 位置 PD 不是棱捕获控制器，需换成边角角度反馈 + 轮速卸载（可复用 G1 已验收的控制律结构）
3. G4 前需做**融合决定**：G4 必须选定一个求解器档位，届时 G2D 与 G3C 都要在该档位上重新验证

## 附：本次新增/遗留的临时文件

诊断与验证脚本 `diag_*.m` / `diag_*_out.txt` 与一次性模型 `Diag_*.slx` 均为本次工作产物，`Cubli_Clean_G2D_EdgeCapture.slx` 未被改动。需要清理请告知。
