%RUN_CUBLI_CLEAN 入口。按 Run 即可。
%
% 这是**发布版**的入口：只保留统一的那台自由接触被控对象。
% 开发目录里还有一批理想支撑台架（棱铰、顶点万向节、辨识模型）与 131 个内部实验/诊断脚本，
% **不在本发布版内**。
%
% 要改的只有下面那两行，模型**不需要重建**——模式、初始姿态、终止时间全部从工作区读取。
%
% 详见 README.md（英文，含快速开始与能力现状）与 docs/（中文，含全部证据与失败记录）。

% ---------------------------------------------------------------------------
% 跑哪个功能
%
%   "edge_balance"          棱上平衡（起始就在棱上）
%   "stand_to_edge"         平放 -> 起立到棱 -> 保持
%   "point_balance"         顶点平衡（起始就在顶点）
%   "edge_to_point"         棱 -> 顶点，只做第二跳
%   "flat_to_point"         平放 -> 棱 -> 顶点 -> 保持，完整起立
%   "flat_to_point_direct"  平放 -> 顶点，一步到位（实验级，未升格）
%   "walk"                  一个面一个面地翻（验收未通过，见 docs/03）
%
% 其中 edge_balance、point_balance、edge_to_point 从"已经平衡好"的姿态开始，
% 因为它们是控制律开发用的；其余四个才是"从平放起立"的完整动作。
% ---------------------------------------------------------------------------
cubliMode = "edge_balance";

% ---------------------------------------------------------------------------
% 控制律怎么处理飞轮转速
%
%   "fast"           不加转速环。方块能平衡，但飞轮转速会**爬到约 950 rad/s** 然后冻结，
%                    随后方块慢慢倾覆、约 4 分钟倒下。这是原始行为，留作参照，
%                    也是**已验收**的那一套。
%   "wheel_stop"     加一个转速积分项，把飞轮稳在 40-60 rad/s。**需要接触求解器 0.25 ms**
%                    （本脚本会自动在块上设置）。
%   "wheel_stop_yaw" 同上，另加柔和的 X/Z 偏航保持。慢速自转减半、寿命约翻倍，
%                    代价是 X/Z 轮以 0.157 rad/s 每秒线性增长（600 s 到 ±84 rad/s）。
%   "edge_low_speed" 棱平衡专用实验预设：轮速积分增益 8e-5，自动选 ODE5 @ 0.125 ms。
%                    从平放起立约 1.17 s；300 s 时 Y 轮约 7.5 rad/s。
%                    求解器敏感性和接触位移限制见 docs/06。
%   "edge_near_zero" 棱平衡专用近零轮速实验预设：KiEdge=1e-3，ODE5 @ 0.125 ms。
%                    连续稳定捕获约 2.28 s；300 s 最末 30 s Y 轮约 -0.73 到 +1.73 rad/s。
%                    接触位移、偏航及步长敏感性仍未解决，见 docs/06。
% ---------------------------------------------------------------------------
cubliPreset = "fast";

% 仿真时长（秒），留空 [] 则用模式自带的默认时长。
% 有些模式默认只有 10-20 s，那**太短，看不见慢速漂移**：转速环的效果大约要一分钟之后
% 才与参照明显分开，而倾角漂移要几分钟才致命。想看那些，填 600。
cubliSeconds = 90;

% ---------------------------------------------------------------------------
setup_cubli();

ov = {'run.balancePreset',cubliPreset};
if ~isempty(cubliSeconds)
    ov = [ov; {'run.edge.duration',cubliSeconds; ...
               'run.point.duration',cubliSeconds; ...
               'run.flatchain.duration',cubliSeconds; ...
               'run.directjump.duration',cubliSeconds}];
    if cubliPreset == "edge_low_speed" || cubliPreset == "edge_near_zero"
        ov = [ov; {'run.standup.duration',cubliSeconds}];
    end
end
P = cubli_mode(cubliMode,ov);

model = build_cubli_clean_cubli();

% 接触求解器步长是**建模型时烤进去的**：build_cubli_clean_cubli 自己调用
% cubli_clean_parameters()，并把数字作为字面量写进 Solver Configuration 块，
% 所以在已经建好的模型上再改 P.sim.localSolver 是无效的。
% 两套带转速环的预设需要 0.25 ms，因此在这里对块设置。
% 后面那句回读断言不是装饰——**一个静默没生效的求解器设置，看起来和"这个设置没影响"一模一样**，
% 而这一条在本项目上浪费过真实时间。
if cubliPreset == "wheel_stop" || cubliPreset == "wheel_stop_yaw"
    set_param([model '/Solver Configuration'], ...
        'MultibodyLocalSolverSampleTime','0.00025');
    assert(strcmp(get_param([model '/Solver Configuration'], ...
        'MultibodyLocalSolverSampleTime'),'0.00025'), ...
        'Cubli:SolverStep','The 0.25 ms solver step did not take.');
    fprintf(['%s preset: contact solver set to 0.25 ms on the block ' ...
        '(it is baked at build time).\n'],cubliPreset);
elseif cubliPreset == "edge_low_speed" || cubliPreset == "edge_near_zero"
    set_param([model '/Solver Configuration'], ...
        'MultibodyLocalSolverChoice','ODE5', ...
        'MultibodyLocalSolverSampleTime','0.000125');
    assert(strcmp(get_param([model '/Solver Configuration'], ...
        'MultibodyLocalSolverChoice'),'ODE5') && ...
        strcmp(get_param([model '/Solver Configuration'], ...
        'MultibodyLocalSolverSampleTime'),'0.000125'), ...
        'Cubli:SolverSetting','The edge low-speed solver setting did not take.');
    fprintf('%s preset: contact solver ODE5 @ 0.125 ms.\n',cubliPreset);
end

fprintf('\nRunning "%s" with preset "%s": %.1f s.\n\n', ...
    cubliMode,cubliPreset,P.run.duration);
assignin('base','P',P);
out = sim(model,'ReturnWorkspaceOutputs','on');

% 把日志放到工作区，这样它们会以名字出现在仿真数据检查器里。
for nm = ["cubli_com","cubli_rate","cubli_R","cubli_penetration", ...
          "cubli_tau_x","cubli_tau_y","cubli_tau_z", ...
          "cubli_wx","cubli_wy","cubli_wz"]
    assignin('base',nm,out.get(nm));
end

% 同时把 cubli_R 传进报告，这样它可以并排打印**姿态倾角**与 com 版倾角。
% 在这台被控对象上这两者不是一个量：asin(com_x/d) 也会看见方块**沿棱滑移**，
% 于是真实姿态还水平时它就读出几度。两个都打印，差别才看得见。
cubli_report = cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
    out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
    out.get('cubli_penetration'),P,out.get('cubli_R'),out.get('cubli_wy'));
assignin('base','cubli_report',cubli_report);

% 长跑会**合法地**判失败，这里说清楚，免得第一次用的人以为坏了。
% 门槛（5°）是按 10-20 s 的验收运行写的；跑到 90 s 或更久时，那条已知的倾角/滑移漂移
% 会自然超过它——而它读的 `com_x` 还有滑移成分（见上面"姿态倾角"那一行的对比）。
% docs/03 里记录了这条漂移本身，以及为什么先到极限的是倾角而不是轮速。
if ~cubli_report.ok && P.run.duration > 30 && ...
        numel(cubli_report.failedChecks) == 1 && ...
        strcmp(cubli_report.failedChecks{1},'tiltOk')
    fprintf(['\n  NOTE: the only failed check is tiltOk, and this run is %.0f s.\n' ...
        '  That gate is written for 10-20 s acceptance runs. Over minutes the cube''s\n' ...
        '  tilt (and the com_x slide the gate reads) drifts past it -- a real, documented\n' ...
        '  limitation, not a broken run. See docs/03_未解决与已证伪.md.\n'], ...
        P.run.duration);
end

% 高度轨迹。只看 com_z 一个数就能判断方块最终停在哪种姿态——
% 顶点 0.12954 / 棱 0.10570 / 平放 0.07464——所以"最后停在某个看似合理的地方"
% 不会被误当成通过。
c = cubli_log_reshape(out.get('cubli_com'),3);
t = out.get('cubli_com').Time(:).';
fprintf('%6s %10s\n','t','com_z');
for tk = 0:max(2,round(t(end)/12)):ceil(t(end))
    k = find(t>=tk,1);
    if isempty(k), break; end
    fprintf('%6.0f %10.5f\n',t(k),c(3,k));
end
fprintf('\ncom_z end %.5f   (vertex 0.12954 / edge 0.10570 / flat 0.07464)\n', ...
    c(3,end));
if c(3,end) > 0.120
    fprintf('=> ended BALANCED ON THE VERTEX\n');
elseif c(3,end) > 0.095
    fprintf('=> ended ON THE EDGE, not the vertex\n');
else
    fprintf('=> ended FLAT\n');
end

% 三只飞轮转速正是两套转速环预设的重点，所以直接打印，而不是留给数据检查器。
g = @(nm) cubli_log_reshape(out.get(nm),1);
wy = g('cubli_wy'); wx = g('cubli_wx'); wz = g('cubli_wz');
fprintf('\nWheel speeds, rad/s (limit %g):\n',P.motor.maxSpeed);
for tk = 0:max(2,round(t(end)/6)):ceil(t(end))
    k = find(t>=tk,1);
    if isempty(k), break; end
    fprintf('%6.0f   X %8.1f   Y %8.1f   Z %8.1f\n', ...
        t(k),wx(k),wy(k),wz(k));
end
