function result = solve_Q3(params)
% solve_Q3 — 问题3：FY1投放3枚烟幕干扰弹，对M1进行干扰
% 使用遗传算法(GA)优化投放策略，使有效遮蔽时间最大化
% 决策变量：θ, v, t_r1, Δt1, t_r2, Δt2, t_r3, Δt3 (8个)
% 输出保存到 result1.xlsx
%
% 输入:
%   params : 参数字典
% 输出:
%   result : 结构体，包含最优解信息

fprintf('\n========== 问题3 ==========\n');

n_uavs = 1;      % FY1
n_bombs = 3;     % 3枚弹
n_vars = 2 + 2 * n_bombs;  % 8个变量

%% 决策变量边界
% [θ, v, t_r1, Δt1, t_r2, Δt2, t_r3, Δt3]
t_impact = norm(params.missiles(1, :)) / params.v_missile;

lb = [0, 70, 0, 0.5, 1, 0.5, 2, 0.5];
ub = [360, 140, 15, 18, 25, 18, 35, 18];

%% 构造目标函数
obj_fun = @(x) objective_Q3(x, params);

%% 使用GA优化
ga_opts = struct();
ga_opts.PopulationSize = 250;
ga_opts.MaxGenerations = 300;
ga_opts.StallGenLimit = 40;

% 先尝试GA
fprintf('阶段1: 遗传算法全局搜索...\n');
try
    [x_opt, f_val] = ga_wrapper(obj_fun, n_vars, lb, ub, params, ...
        n_uavs, n_bombs, ga_opts);
catch
    fprintf('GA工具箱不可用，使用fmincon多起点搜索...\n');
    % 回退方案：多起点fmincon
    [x_opt, f_val] = multistart_fmincon(obj_fun, n_vars, lb, ub, params, ...
        n_uavs, n_bombs);
end

T_shield_ga = -f_val;
fprintf('GA最优遮蔽时间: %.4f s\n', T_shield_ga);

%% 用fmincon精化
fprintf('\n阶段2: fmincon局部精化...\n');
con_fun = @(x) constraints(x, params, n_uavs, n_bombs);
opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
    'MaxIterations', 300, 'MaxFunctionEvaluations', 5000);

try
    [x_refined, f_refined] = fmincon(obj_fun, x_opt, [], [], [], [], ...
        lb, ub, con_fun, opts);
    if f_refined < f_val
        x_opt = x_refined;
        f_val = f_refined;
    end
catch
    fprintf('fmincon精化失败，使用GA结果。\n');
end

T_shield_final = -f_val;

%% 提取结果
theta = x_opt(1);
v_speed = x_opt(2);
t_releases = [x_opt(3), x_opt(5), x_opt(7)];
delta_ts = [x_opt(4), x_opt(6), x_opt(8)];

% 确保时间顺序
[t_releases, sort_idx] = sort(t_releases);
delta_ts = delta_ts(sort_idx);

fprintf('\n=== 问题3最优策略 ===\n');
fprintf('航向角: θ = %.2f°\n', theta);
fprintf('飞行速度: v = %.2f m/s\n', v_speed);
fprintf('总有效遮蔽时长: T = %.4f s\n', T_shield_final);

uav_init = params.uavs(1, :);
th_rad = deg2rad(theta);
uav_vel = [v_speed * cos(th_rad), v_speed * sin(th_rad), 0];

release_positions = zeros(3, 3);
det_positions = zeros(3, 3);
t_dets = zeros(1, 3);

for b = 1:3
    release_positions(b, :) = uav_traj(uav_init, theta, v_speed, t_releases(b));
    det_positions(b, :) = bomb_traj(release_positions(b, :), uav_vel, ...
        t_releases(b), t_releases(b) + delta_ts(b));
    t_dets(b) = t_releases(b) + delta_ts(b);

    fprintf('干扰弹%d: tr=%.2fs, Δt=%.2fs, td=%.2fs\n', b, ...
        t_releases(b), delta_ts(b), t_dets(b));
    fprintf('  投放点: (%.1f, %.1f, %.1f) m\n', release_positions(b, :));
    fprintf('  起爆点: (%.1f, %.1f, %.1f) m\n', det_positions(b, :));
end

%% 返回结果
result = struct();
result.T_shield = T_shield_final;
result.theta = theta;
result.v = v_speed;
result.t_releases = t_releases;
result.delta_ts = delta_ts;
result.release_positions = release_positions;
result.det_positions = det_positions;
result.t_dets = t_dets;
result.x_opt = x_opt;
end

%% 备用：多起点fmincon
function [x_best, f_best] = multistart_fmincon(obj_fun, n_vars, lb, ub, ...
    params, n_uavs, n_bombs)
con_fun = @(x) constraints(x, params, n_uavs, n_bombs);
opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
    'MaxIterations', 200, 'MaxFunctionEvaluations', 3000);

n_starts = 10;
x_best = [];
f_best = inf;

for s = 1:n_starts
    x0 = lb + rand(1, n_vars) .* (ub - lb);
    x0(1) = rand * 360;  % 角度在[0,360]均匀分布
    try
        [x_opt, f_val] = fmincon(obj_fun, x0, [], [], [], [], lb, ub, con_fun, opts);
        if f_val < f_best
            f_best = f_val;
            x_best = x_opt;
        end
    catch
        continue;
    end
end
end
