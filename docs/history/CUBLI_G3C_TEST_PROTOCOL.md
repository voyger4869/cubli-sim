# G3C：三飞轮点平衡闭环验收

## 当前目标

G3C 是理想顶点支撑上的三飞轮、双轴离散闭环试验。它不包含物理点接触，也不包含行走；当前验收只回答一个问题：从小初始姿态偏差出发，控制器是否能在不持续饱和的前提下把 Roll/Pitch 收敛到零附近。

当前 G3C 使用质心平衡轮心布局下重新辨识的控制矩阵。用户实测 10 s：`qPeak=[0.000872885, 0.000698308] rad`、`qdPeak=[0.00468416, 0.00374734] rad/s`、`tauPeak=[0.000832607, 0.0022619, 0.00142929] N m`，通过理想顶点支撑点平衡验收。

## 已接入的实际链路

```text
Point Roll/Pitch 位置、速度
  -> Point Controller（二维 PD + 2×3 阻尼分配）
  -> 三路 2 ms Unit Delay
  -> X/Y/Z 飞轮关节力矩输入
```

控制矩阵来自修正顶点几何和质心平衡轮心布局后的 G3I 脉冲试验，使用的是角速度增量除以力矩角冲量，而不是此前不可靠的 0.1 s 末端角度差。

## 当前默认条件

| 项目 | 值 |
|---|---:|
| 初始 Roll | +0.05° |
| 初始 Pitch | -0.04° |
| 控制周期 | 0.002 s |
| 连续力矩限制 | ±0.06 N m / 飞轮 |
| 标准验收仿真时长 | 10 s |
| `Kp` / `Kd` | 200 / 14 |

这些增益通过当前 10 s 理想顶点支撑试验；在更换惯量、轮位、采样周期或执行器限制后必须重新验收。

## 运行方式

在 `run_cubli_clean.m` 顶部保留：

```matlab
mode = "point_balance";
```

运行：

```matlab
run_cubli_clean
```

成功时将生成 `Cubli_Clean_G3C_PointBalance.slx` 并写出：

- `g3c_roll`、`g3c_pitch`
- `g3c_roll_rate`、`g3c_pitch_rate`
- `g3c_tau_x`、`g3c_tau_y`、`g3c_tau_z`

## 10 秒验收命令

先进行几何门槛检查。G3C 起始瞬间，质心应位于顶点正上方，即世界坐标
`x≈0`、`y≈0`、`z≈0.129904 m`。若前两项不接近零，控制器必然需要先抵消恒定重力力矩，不能开始调增益：

```matlab
format long g
g3c_com.Data(:,1)
```

只有该几何检查通过后，再检查控制状态：

```matlab
qf = [g3c_roll.Data(end), g3c_pitch.Data(end)];
qdf = [g3c_roll_rate.Data(end), g3c_pitch_rate.Data(end)];
qPeak = [max(abs(g3c_roll.Data)), max(abs(g3c_pitch.Data))];
qdPeak = [max(abs(g3c_roll_rate.Data)), max(abs(g3c_pitch_rate.Data))];
tauPeak = [max(abs(g3c_tau_x.Data)), ...
           max(abs(g3c_tau_y.Data)), ...
           max(abs(g3c_tau_z.Data))];

fprintf('qf=[%.6g %.6g], qdf=[%.6g %.6g], qPeak=[%.6g %.6g], qdPeak=[%.6g %.6g], tauPeak=[%.6g %.6g %.6g]\\n', ...
    qf, qdf, qPeak, qdPeak, tauPeak)
```

## 如何判断第一次结果

| 现象 | 判定与下一动作 |
|---|---|
| `qf` 和 `qdf` 接近零，全程角度/角速度保持小，三路力矩未长期顶住 ±0.06 | G3C 点平衡 10 s 验收通过。当前用户结果已满足。 |
| 姿态立刻单向发散，三路很快饱和 | 记录力矩符号与状态；优先检查分配符号，不盲目增益。 |
| 姿态振荡、频率稳定 | 降低 `Kd` 或 `Kp` 后复测。 |
| 只有一只轮转 | 检查三路 `tauPeak`；这是连线或分配问题。 |
| 模型构建/编译报错 | 贴完整错误；不要自行重接物理端口。 |

请反馈完整 `fprintf` 输出，以及动画中方块是否始终绕同一顶点保持平衡。
