function result = solve_Q2(params)
% solve_Q2 — 问题2：单枚干扰弹的最优投放策略优化
% 决策变量：航向角θ、飞行速度v、投放时刻t_r、起爆延迟Δt
% 目标：最大化对M1的有效遮蔽时长
% 方法：网格搜索 + fmincon局部精化
%
% 输入:
%   params : 参数字典
% 输出:
%   result : 结构体，包含最优解及遮蔽时长

fprintf('\n========== 问题2 ==========\n');

uav_id = 1;
missile_id = 1;
uav_init = params.uavs(uav_id, :);

%% 第一阶段：网格搜索（粗搜索）
fprintf('阶段1: 网格搜索...\n');

% 搜索范围
theta_range = 0:5:355;          % 航向角 (度)
v_range = 70:5:140;             % 速度 (m/s)
% 动态确定投放时间和起爆延迟范围
t_release_range = 0:1:15;       % 投放时刻 (s)
delta_t_range = 0.5:0.5:18;     % 起爆延迟 (s)

n_total = length(theta_range) * length(v_range) * ...
    length(t_release_range) * length(delta_t_range);
fprintf('总网格点数: %d\n', n_total);

best_T = -inf;
best_x = zeros(1, 4);

% 预计算导弹撞击时间
t_impact = norm(params.missiles(missile_id, :)) / params.v_missile;

count = 0;
for th = theta_range
    for sp = v_range
        for tr = t_release_range
            for dt_val = delta_t_range
                count = count + 1;

                % 快速约束检查
                % 起爆必须在撞击前且不低于地面
                t_det = tr + dt_val;
                if t_det >= t_impact || t_det <= 0
                    continue;
                end

                % 计算起爆点z坐标（快速检查）
                uav_pos = uav_traj(uav_init, th, sp, tr);
                th_rad = deg2rad(th);
                uav_vel = [sp * cos(th_rad), sp * sin(th_rad), 0];
                det_pos = bomb_traj(uav_pos, uav_vel, tr, tr + dt_val);
                if det_pos(3) <= 0
                    continue;
                end

                % 计算遮蔽时间
                T = shielding_time(missile_id, uav_id, th, sp, tr, dt_val, params);

                if T > best_T
                    best_T = T;
                    best_x = [th, sp, tr, dt_val];
                end
            end
        end
    end
    % 每完成一个角度打印进度
    if mod(count, 10000) < length(v_range)*length(t_release_range)*length(delta_t_range)
        fprintf('  进度: %.1f%%, 当前最优 T = %.4f s\n', ...
            100*count/n_total, best_T);
    end
end

fprintf('网格搜索完成: T_max = %.4f s\n', best_T);
fprintf('最优参数: θ=%.1f°, v=%.1f m/s, tr=%.1f s, Δt=%.1f s\n', best_x);

%% 第二阶段：fmincon局部精化
fprintf('\n阶段2: fmincon局部优化...\n');

% 目标函数（最小化负的遮蔽时间）
obj_fun = @(x) -shielding_time(missile_id, uav_id, x(1), x(2), x(3), x(4), params);

% 多起点优化
n_starts = 5;
opts = optimoptions('fmincon', 'Display', 'off', ...
    'Algorithm', 'sqp', 'MaxIterations', 200, 'MaxFunctionEvaluations', 5000);

x_best_global = best_x;
f_best_global = -best_T;

for s = 1:n_starts
    % 在最优网格点附近随机扰动作为起点
    perturb = [rand*20-10, rand*10-5, rand*2-1, rand*2-1];
    x0 = best_x + perturb;
    x0(1) = mod(x0(1), 360);  % 角度取模
    x0(2) = max(70, min(140, x0(2)));  % 速度范围
    x0(3) = max(0, x0(3));  % 投放时刻非负
    x0(4) = max(0.1, x0(4)); % 延迟为正

    % 变量下界和上界
    lb = [0, 70, 0, 0.1];
    ub = [360, 140, t_impact, t_impact];

    try
        [x_opt, f_val] = fmincon(obj_fun, x0, [], [], [], [], lb, ub, ...
            @(x) nl_constraints(x, params, missile_id, uav_id), opts);
        if -f_val > f_best_global
            f_best_global = -f_val;
            x_best_global = x_opt;
        end
    catch
        fprintf('  起点 %d 优化失败，跳过。\n', s);
    end
end

best_x = x_best_global;
best_T = f_best_global;

fprintf('最终优化结果: T_max = %.4f s\n', best_T);

%% 计算最终策略的详细结果
th_opt = best_x(1);
sp_opt = best_x(2);
tr_opt = best_x(3);
dt_opt = best_x(4);

uav_pos_opt = uav_traj(uav_init, th_opt, sp_opt, tr_opt);
th_rad = deg2rad(th_opt);
uav_vel_opt = [sp_opt * cos(th_rad), sp_opt * sin(th_rad), 0];
det_pos_opt = bomb_traj(uav_pos_opt, uav_vel_opt, tr_opt, tr_opt + dt_opt);

fprintf('\n=== 问题2最优策略 ===\n');
fprintf('航向角:     θ = %.2f°\n', th_opt);
fprintf('飞行速度:   v = %.2f m/s\n', sp_opt);
fprintf('投放时刻:   tr = %.2f s\n', tr_opt);
fprintf('起爆延迟:   Δt = %.2f s\n', dt_opt);
fprintf('起爆时刻:   td = %.2f s\n', tr_opt + dt_opt);
fprintf('投放点:     (%.2f, %.2f, %.2f) m\n', uav_pos_opt);
fprintf('起爆点:     (%.2f, %.2f, %.2f) m\n', det_pos_opt);
fprintf('有效遮蔽时长: T = %.4f s\n', best_T);

%% 返回结果
result = struct();
result.T_shield = best_T;
result.theta = th_opt;
result.v = sp_opt;
result.t_release = tr_opt;
result.delta_t = dt_opt;
result.uav_pos_release = uav_pos_opt;
result.det_pos = det_pos_opt;
result.t_det = tr_opt + dt_opt;
result.t_expire = tr_opt + dt_opt + params.T_smoke_eff;
end

%% 非线性约束函数
function [c, ceq] = nl_constraints(x, params, missile_id, uav_id)
% 约束条件：
% 1. 起爆点z坐标 > 0（不能在地下起爆）
% 2. 起爆时刻 < 导弹到达假目标的时刻
% 3. 投放时刻 >= 0
% 4. 起爆延迟 > 0

th = x(1); sp = x(2); tr = x(3); dt_val = x(4);

uav_init = params.uavs(uav_id, :);
uav_pos = uav_traj(uav_init, th, sp, tr);
th_rad = deg2rad(th);
uav_vel = [sp * cos(th_rad), sp * sin(th_rad), 0];
det_pos = bomb_traj(uav_pos, uav_vel, tr, tr + dt_val);

t_impact = norm(params.missiles(missile_id, :)) / params.v_missile;

% 不等式约束 c <= 0
c = zeros(3, 1);
c(1) = -det_pos(3);                    % z_det > 0 → -z_det < 0
c(2) = (tr + dt_val) - t_impact;       % t_det < t_impact
c(3) = -tr;                             % tr >= 0 → -tr <= 0

ceq = [];  % 无等式约束
end
