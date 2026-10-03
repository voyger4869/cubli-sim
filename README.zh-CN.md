# Cubli Sim

![三只反作用轮与地面接触的 Cubli 矢量示意图](assets/cubli-hero.svg)

**基于 MATLAB、Simulink 与 Simscape Multibody 的三维反作用轮方块仿真。**方块通过三只飞轮和控制器在棱或顶点上保持平衡，也能从平放状态物理起立，并尝试逐面行走。模型包含自由刚体和地面接触，动作由控制器与动力学仿真产生，没有预录动画。

**语言：**[简体中文](README.zh-CN.md) · [English](README.md)

> **项目定位：**这是仿真和控制研究项目。下文结果只适用于写明的模型和求解器设置，不能直接当作实物性能。详细架构、测量记录以及失败的方案见 [`docs/`](docs/)。

---

## 快速运行

### 1. 环境

- MATLAB **R2024b**（当前开发和验证所用版本）
- **Simulink**、**Simscape**、**Simscape Multibody**

克隆或下载仓库：

```bash
git clone https://github.com/voyger4869/cubli-sim.git
cd cubli-sim
```

将 MATLAB 的**当前文件夹**设为仓库根目录，即包含 `setup_cubli.m` 的目录。**第一次建议先跑短试验：**打开 [`src/core/run_cubli_clean.m`](src/core/run_cubli_clean.m)，把模式、预设、时长设为：

```matlab
cubliMode    = "edge_balance";
cubliPreset  = "fast";
cubliSeconds = 10;
```

然后在 MATLAB 命令窗口运行：

```matlab
setup_cubli
run_cubli_clean
```

入口脚本本身也会调用 `setup_cubli`；单独执行第一行可以明确完成路径设置，直接运行验证函数时也需要先设置路径。脚本会打开从源码生成的 Simulink 模型、运行仿真、打印报告，并把 `cubli_com`、`cubli_R`、`cubli_wx` / `cubli_wy` / `cubli_wz` 和 `cubli_report` 等放入 MATLAB 工作区。生成的模型和仿真缓存不提交到 Git。

### 2. 选择要看的功能

修改入口顶部的模式、预设和时长，然后重新运行；路线模式还需设置 `cubliRoute`：

| 目标 | `cubliMode` | `cubliPreset` | `cubliSeconds` |
|---|---|---|---:|
| 看基础棱平衡 | `"edge_balance"` | `"fast"` | `10` |
| 从平放起立到棱，并让飞轮保持在零附近 | `"stand_to_edge"` | `"edge_near_zero"` | `300` |
| 看完整的平放 → 棱 → 顶点两段起立 | `"flat_to_point"` | `"fast"` | `[]` |

`[]` 表示使用该模式在参数文件中的默认时长。**目前 `cubliSeconds` 的覆盖范围与模式有关：**非空时可覆盖 `edge_balance`、`point_balance`、`flat_to_point`、`flat_to_point_direct`；对于 `stand_to_edge`，只有选择 `edge_low_speed` 或 `edge_near_zero` 时才会覆盖。其他模式继续使用各自默认时长。入口会自动为相应的轮速预设设置并核对接触求解器。

如果想看**一台已生成的模型**在多种模式间切换、而不在两次仿真之间重新生成 `.slx`，先执行 `setup_cubli`，再运行 `cubli_demo`。普通入口 `run_cubli_clean` 每次运行都会重新生成模型；用户不需要手动搭建或修改 `.slx`。

## 目前实现了什么

所有模式共用同一台自由接触模型。表中的“通过”指当前仿真的验收判据，不代表无限时间稳定。

| 功能 | 模式 | 当前状态 |
|---|---|---|
| 从已在棱上的姿态开始平衡 | `edge_balance` | 短时验收通过 |
| 平放 → 起立到棱 → 保持 | `stand_to_edge` | 通过 |
| 从已在顶点上的姿态开始平衡 | `point_balance` | 10 秒验收通过 |
| 棱 → 顶点 | `edge_to_point` | 通过 |
| **平放 → 棱 → 顶点 → 保持** | `flat_to_point` | 通过；完整两段起立 |
| 平放直接跳到顶点 | `flat_to_point_direct` | 当前判据通过，仍属实验方案 |
| 逐面行走 | `walk` | 能移动，但面数与接触穿透验收不通过 |
| 字符串路线行走 | `walk_route` | `RLUD`、`RRRRRR` 和拐弯 `RU` 均逐步完成；接触穿透验收未通过，仍属实验模式 |
| 冲量式平放 → 顶点 | `stand_to_point` | 未通过；飞轮超出转速预算 |

`verify_cubli` 使用已验收的 `fast` 预设检查全部八种模式：**六项通过，`walk` 和 `stand_to_point` 为已知失败项。**准确边界见[项目状态](docs/01_项目状态.md)与[失败分析](docs/03_未解决与已证伪.md)。

### 字符串路线行走

在 [`src/core/run_cubli_clean.m`](src/core/run_cubli_clean.m) 中设置 `cubliMode = "walk_route"`，并将 `cubliRoute` 写成由 `R`、`L`、`U`、`D` 组成的字符串，例如 `"RLUD"` 或 `"RRRRRR"`。`R/L` 分别沿地面固定世界坐标 `+X/-X`，`U/D` 沿 `+Y/-Y`；一个字符表示翻过一个面。路线长度为 1–64 步，大小写均可；时长默认按每步 4 秒加 2 秒余量自动计算。`cubliSeconds` 不覆盖该模式的时长。

控制器等当前一步转过约 90°、飞轮减速、方块落平且角速度变小，再执行下一个字符。转向时会根据方块当前姿态重新选飞轮。工作区的 `cubli_route_state` 记录 `[已完成步数; 阶段; 故障码]`，`cubli_report` 同时核对逐步位移、终点误差、落稳与接触穿透。可运行 `verify_walk_route` 复现三个短路线。**现有接触模型的峰值穿透约 2.4–2.5 mm，超过原验收门槛；路线动作完成不代表整体验收通过。**设计和逐项结果见[路线行走开发记录](docs/08_字符串路线行走开发记录.md)。

## 棱平衡飞轮转速预设

在入口脚本中用 `cubliPreset` 选择。下列数字针对相应求解器下棱平衡阶段的 **Y 飞轮**。`edge_low_speed` 和 `edge_near_zero` 仅支持 `edge_balance`、`stand_to_edge`。

| 预设 | 已观察到的行为 | 接触求解器与适用范围 |
|---|---|---|
| `fast` | 捕获快；Y 轮可能升至约 950 rad/s 后趋于平台 | 原始参照行为 |
| `wheel_stop` | 既有棱模式记录中 Y 轮约 40–60 rad/s | 局部步长 0.25 ms |
| `wheel_stop_yaw` | 增加柔和偏航保持；X/Z 轮转速会随时间累积 | 局部步长 0.25 ms |
| `edge_low_speed` | 平放起立后 30–300 秒，Y 轮约 6–8 rad/s | 实验预设；ODE5 @ 0.125 ms |
| `edge_near_zero` | 300 秒试验的末 30 秒，Y 轮为 **−0.73～+1.73 rad/s**，均值 **+0.56 rad/s**；连续稳定捕获用时 **2.278 秒** | 实验预设；ODE5 @ 0.125 ms |

近零结果**不等于严格收敛到零**：偏航和接触位置仍会漂移，改变接触求解器步长也会改变轮速结果。改进过程、失败对照和数值敏感性见[低轮速方案](docs/06_棱平衡低轮速控制设计.md)与[近零轮速开发全过程](docs/07_近零轮速改进开发全过程.md)。

## 如何验证

在仓库根目录运行 `setup_cubli` 后，可执行：

```matlab
verify_cubli                                % 八种模式；其中两项为已知失败
verify_edge_low_speed("stand_to_edge",30)   % 低轮速快速检查
verify_edge_near_zero("stand_to_edge",300)  % 近零轮速长时检查
verify_edge_near_zero("edge_balance",120)   % 从棱上直接起步
verify_walk_route                            % RLUD、RRRRRR、RU；路线实验
```

近零轮速验证脚本会核对求解器块、连续稳定捕获、随棱姿态、Y 轮末段统计和**三轮合成转速**。300 秒验证需要较长运行时间。若要比较修改前后的原始行为，可使用 [`src/verification/`](src/verification/) 中的 `verify_preset_snapshot`、`verify_preset_compare`。

## 项目如何跑起来

```mermaid
flowchart LR
    A["run_cubli_clean<br/>模式 · 预设 · 时长"] --> B["cubli_mode<br/>初始状态 + 参数"]
    B --> C["build_cubli_clean_cubli<br/>生成 Simulink 模型"]
    C --> D["sim<br/>地面接触 + 飞轮 + 控制器"]
    D --> E["cubli_run_report<br/>信号 + 验收报告"]
```

| 路径 | 作用 |
|---|---|
| [`setup_cubli.m`](setup_cubli.m) | 将实际存在的源码文件夹加入 MATLAB 路径 |
| [`src/core/run_cubli_clean.m`](src/core/run_cubli_clean.m) | 主入口，选择模式、预设和时长 |
| [`src/core/cubli_clean_parameters.m`](src/core/cubli_clean_parameters.m) | 物理参数、控制增益、模式默认值及取值依据 |
| [`src/core/cubli_mode.m`](src/core/cubli_mode.m) | 生成所选模式的初始状态与终止时间 |
| [`src/core/build_cubli_clean_cubli.m`](src/core/build_cubli_clean_cubli.m) | 编程生成统一的自由接触模型和控制器 |
| [`src/verification/`](src/verification/) | 能力验收与回归对照脚本 |

发布仓库包含可运行的模型生成代码和验证脚本。完整开发副本还保留实验台架和历史实验脚本；历史文档中引用的部分脚本并未全部放进本发布仓库。

## 文档索引

| 文档 | 主要内容 |
|---|---|
| [01 · 项目状态](docs/01_项目状态.md) | 已实现能力、当前进度和开放问题 |
| [02 · 测试协议](docs/02_测试协议.md) | 验收门槛与测量证据 |
| [03 · 未解决与已证伪](docs/03_未解决与已证伪.md) | 失败方案、更正记录和常见误判 |
| [04 · 测量记录](docs/04_测量记录.md) | 既有实验数据 |
| [05 · 架构与开发指南](docs/05_架构与开发指南.md) | 建模、数据流、控制逻辑和继续开发方式 |
| [06 · 棱平衡低轮速控制设计](docs/06_棱平衡低轮速控制设计.md) | 低轮速预设与求解器敏感性 |
| [07 · 近零轮速改进开发全过程](docs/07_近零轮速改进开发全过程.md) | 每轮尝试、问题、修正与发布验收 |
| [08 · 字符串路线行走开发记录](docs/08_字符串路线行走开发记录.md) | 路线语法、转向控制、仿真结果与未解决的接触问题 |

当前接触模型**尚无跨局部求解步长的数值收敛证据**。长时间仿真仍有偏航及接触位置漂移；阅读轮速结果时必须同时看这些状态和求解器配置。项目尚未完成实物控制器验证。

## 许可证

[MIT](LICENSE)。
