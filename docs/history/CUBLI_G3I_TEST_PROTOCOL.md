# G3I：三飞轮双轴小信号辨识试验

## 目的与边界

G3I 的目的不是平衡 Cubli，而是在相同初始条件下测量每只飞轮的短脉冲对两个顶点倾角及其角速度的影响。输出用于重新标定 `P.control.point.responseMatrix`，之后才可开发 G3C 点平衡闭环。

G3I 使用理想顶点支撑（两个 Revolute Joint），没有物理地面点接触；因此本阶段不评价穿模、起立或行走。

## 当前试验参数

> 2026-09-26：顶点姿态已修正，初始立方体壳体质心为 `[0,0,0.129903810568] m`。随后又发现三只飞轮质量位置不平衡，已重新布置轮心使等质量飞轮的质心落在方块中心；本协议要求从 ZERO 重新开始，之前所有 G3I 矩阵均无效。

参数均在 `cubli_clean_parameters.m` 中：

| 参数 | 值 |
|---|---:|
| 初始 Roll / Pitch | 0° / 0° |
| 脉冲幅值 | +0.002 N m |
| 脉冲起点 | 0.010 s |
| 脉冲宽度 | 0.005 s |
| 停止时间 | 0.040 s |
| 最大步长 | 0.002 s |

## 如何运行

编辑 `run_cubli_clean.m` 顶部两行：

```matlab
mode = "point_ident";
identificationAxis = "ZERO";  % 之后依次改为 "X"、"Y"、"Z"
```

每次只改变 `identificationAxis`，运行 `run_cubli_clean`。顺序必须为：

1. `ZERO`：零输入基线；
2. `X`：仅 X 飞轮脉冲；
3. `Y`：仅 Y 飞轮脉冲；
4. `Z`：仅 Z 飞轮脉冲。

为检查重复性，每个模式至少运行三次。每次运行会重新生成同一个 `Cubli_Clean_G3I_PointIdentification.slx`，因此请在每次运行后立刻保存需要的数值到工作区或文件，避免被下一次试验覆盖。

## 运行后必须检查的量

模型会把以下 `timeseries` 放入 MATLAB 基础工作区：

| 变量 | 含义 | 单位 |
|---|---|---|
| `g3i_roll` | Roll 角度 | rad |
| `g3i_pitch` | Pitch 角度 | rad |
| `g3i_roll_rate` | Roll 角速度 | rad/s |
| `g3i_pitch_rate` | Pitch 角速度 | rad/s |
| `g3i_tau_x` | X 飞轮命令 | N m |
| `g3i_tau_y` | Y 飞轮命令 | N m |
| `g3i_tau_z` | Z 飞轮命令 | N m |

先验证力矩通道。对于 X 试验，应只看到 X 通道在 0.010–0.015 s 的 0.002 N m 平台，Y/Z 全程为零：

```matlab
[max(abs(g3i_tau_x.Data)), max(abs(g3i_tau_y.Data)), max(abs(g3i_tau_z.Data))]
```

预期输出：

```text
X 试验: [0.002, 0, 0]
Y 试验: [0, 0.002, 0]
Z 试验: [0, 0, 0.002]
ZERO : [0, 0, 0]
```

再记录终点四状态。每次运行后执行：

```matlab
g3i_end = [g3i_roll.Data(end), g3i_pitch.Data(end), ...
           g3i_roll_rate.Data(end), g3i_pitch_rate.Data(end)]
```

请保存 ZERO、X、Y、Z 各三次的 `g3i_end`。后续计算每一轮的净响应时，用每次脉冲结果减去对应的 ZERO 结果：

```matlab
deltaX = g3i_end_X - g3i_end_ZERO;
```

## 应观察并反馈的现象

1. **命令正确性**：每次只应有一只飞轮获得非零脉冲；若三路同时非零、脉冲时刻错误或幅值错误，停止，不计算响应矩阵。
2. **可重复性**：相同轴三次的 `g3i_end` 符号必须相同，量级应接近。若相差很大，说明初值或求解过程不一致。
3. **二维耦合**：每只轮都可能同时影响 Roll 和 Pitch；这正常。不能仅凭“看起来主要沿某方向动”就把它当成单轴电机。
4. **基线**：ZERO 理想情况下接近零；若在 0.040 s 内已明显发散，应缩短窗口，而不是提高脉冲力矩。
5. **动画**：在 40 ms 内，肉眼位移会很小；本试验以日志数值验收，不以明显动画为验收条件。

## 失败时的最小反馈包

若不能运行，请完整贴出报错。若能运行但行为异常，请贴出：

```matlab
[max(abs(g3i_tau_x.Data)), max(abs(g3i_tau_y.Data)), max(abs(g3i_tau_z.Data))]
g3i_end
```

并说明 `identificationAxis` 的值，以及 Mechanics Explorer 中哪一只飞轮转动、方块是否保持同一顶点支撑。
