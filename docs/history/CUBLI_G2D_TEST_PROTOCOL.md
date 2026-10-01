# G2D：面接触到棱捕获试验

## 目标

从平放的自由 Cubli 开始，通过真实地面接触和 Y 飞轮驱动翻转，在前侧棱附近捕获。当前第一道门是连续仿真 5 s、不出现 `Free Body has a degenerate mass distribution`，并观察方块是否留在地面上方、在棱附近受控回摆。

这个 G2D 回归模型使用最新的质心平衡三飞轮位置，复用 G2C 的 X 位置捕获控制器。它是 G2C 长时接触故障的隔离试验；尚未通过前，不标记为棱平衡完成。

## 运行

在 `run_cubli_clean.m` 顶部设置：

```matlab
mode = "edge_capture";
```

运行：

```matlab
run_cubli_clean
```

构建器会生成 `Cubli_Clean_G2D_EdgeCapture.slx`，设定 5 s，并使用 `g2d_` 前缀记录：

- `g2d_cube_com`：世界坐标质心
- `g2d_cube_angle`：方块相对世界的角度
- `g2d_wheel_speed`：Y 飞轮转速
- `g2d_applied_torque`：实际 Y 飞轮力矩

## 运行后反馈

> **2026-09-26 更正**：此前给出的读数片段是**错的**。`To Workspace` 存的是 `3×1×N` 数组（实测 `size(g2d_cube_com.Data) = [3 1 5001]`），`min(c(:,1))` 在它上面取不到 COM 的 x 范围，打印出的 `x=[0.00000 0.07464]` 与真实范围无关。且 `g2d_cube_angle` 来自 `SenseAngle`，无符号且截断在 [0,180]，`max(abs(angle))` 在方块持续旋转时会绕回。

正确的读法：

```matlab
c = reshape(g2d_cube_com.Data, 3, numel(g2d_cube_com.Time));
fprintf('duration=%.3f, COM x=[%.5f %.5f], y=[%.5f %.5f], z=[%.5f %.5f]\n', ...
    g2d_cube_com.Time(end), min(c(1,:)), max(c(1,:)), ...
    min(c(2,:)), max(c(2,:)), min(c(3,:)), max(c(3,:)));
```

穿透不能用 COM 高度判断（棱姿态下 COM 在 0.105 m，会撞地的是前下角点）。`cubli_contact_report.m` 已实现精确的最低顶点算法，G2D 接入后应直接引用其判定而非手算。

请同时说明 Mechanics Explorer 中方块是否穿过地面、是否停留在前侧棱附近或继续翻滚。若仿真报错，请贴完整错误文本。

## 已知的崩溃与其真实原因

延长到 10 s 会出现 `Free Body has a degenerate mass distribution on its follower side`。**它与时长设置无关**：模型每次都在 t≈9.3–9.4 s 崩，5 s 只是跑不到。直接原因是控制器里的轮速限幅条件符号写反（详见 [`CUBLI_PROJECT_STATUS.md`](CUBLI_PROJECT_STATUS.md) 的 G2D 崩溃小节），限幅器从未生效、轮速发散到 9214 rad/s 以上。该修复尚未改入源码。

**不要再把"5 s 不报错"当作通过。** 那只是因为 5 s 短于崩溃时刻。

## 当前限制

该控制器仍使用 X 位置与速度构成的捕获反馈。**它不是棱捕获控制器**：没有边角角度反馈，也没有轮速卸载，因此持续输出正力矩直到轮速无界增长。实测方块在 0.5 s 确实抬到棱高（z=0.1057，理论 0.1061），随后被陀螺反作用推着在地上翻滚（角度峰值 1.196 rad、COM x 从 +0.117 到 −0.080、Y 方向漂移 18.6 cm）。

修好限幅器只会解除崩溃，不会让它变成有效的捕获。角度反馈与轮速卸载属于控制器重做，是独立的一步。
