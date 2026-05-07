function result = solve_Q4(params)
% solve_Q4 — 问题4：FY1、FY2、FY3各投放1枚烟幕干扰弹，对M1进行干扰
% 使用遗传算法(GA)优化投放策略
% 决策变量：θ1,v1,t_r1,Δt1, θ2,v2,t_r2,Δt2, θ3,v3,t_r3,Δt3 (12个)
% 输出保存到 result2.xlsx
%
% 输入:
%   params : 参数字典
% 输出:
%   result : 结构体，包含最优解信息

fprintf('\n========== 问题4 ==========\n');

n_uavs = 3;      % FY1, FY2, FY3
n_bombs = 1;     % 每架各1枚弹
n_vars = n_uavs * (2 + 2 * n_bombs);  % 12个变量

%% 决策变量边界
% [θ1, v1, t_r1, Δt1, θ2, v2, t_r2, Δt2, θ3, v3, t_r3, Δt3]
t_impact = norm(params.missiles(1, :)) / params.v_missile;

lb = zeros(1, n_vars);
ub = zeros(1, n_vars);
for k = 1:n_uavs
    base = (k-1) * 4;
    lb(base + 1) = 0;      % θ
    lb(base + 2) = 70;     % v
    lb(base + 3) = 0;      % t_r
    lb(base + 4) = 0.5;    % Δt

    ub(base + 1) = 360;
    ub(base + 2) = 140;
    ub(base + 3) = 30;     % 较远的UAV需要更长时间
    ub(base + 4) = 25;
end

%% 构造目标函数
obj_fun = @(x) objective_Q4(x, params);

%% 第一阶段：启发式初始化
% 对每架无人机，利用Q2的思路独立求解，再用GA联合优化
fprintf('阶段1: 独立启发式初始化...\n');
init_guess = zeros(1, n_vars);

for k = 1:n_uavs
    % 确定朝向M1来袭方向的大致航向
    uav_init = params.uavs(k, :);
    % M1从(20000,0,2000)飞向原点，在xy平面从+x向-x
    % UAV需要飞向M1轨迹和目标的连线之间
    % 粗略指向真目标和M1之间的区域
    target_region = [10000, 100, 0];  % 中间区域
    dir_vec = target_region(1:2) - uav_init(1:2);
    th_guess = rad2deg(atan2(dir_vec(2), dir_vec(1)));

    init_guess((k-1)*4 + 1) = th_guess;
    init_guess((k-1)*4 + 2) = 120;   % 中速
    init_guess((k-1)*4 + 3) = 2 + k*1;  % 错开投放
    init_guess((k-1)*4 + 4) = 5;
end

%% 第二阶段：GA全局优化
fprintf('阶段2: GA联合优化...\n');
ga_opts = struct();
ga_opts.PopulationSize = 300;
ga_opts.MaxGenerations = 400;
ga_opts.StallGenLimit = 60;

% 构造包含启发式解的初始种群
init_pop = zeros(ga_opts.PopulationSize, n_vars);
init_pop(1, :) = init_guess;
for i = 2:ga_opts.PopulationSize
    % 在启发式解周围随机采样
    perturb = randn(1, n_vars) .* [30, 10, 3, 2, 30, 10, 3, 2, 30, 10, 3, 2];
    init_pop(i, :) = init_guess + perturb;
    % 裁剪到边界内
    init_pop(i, :) = max(lb, min(ub, init_pop(i, :)));
    init_pop(i, 1:4:end) = mod(init_pop(i, 1:4:end), 360);
end

ga_opts.InitialPopulation = init_pop;

try
    [x_opt, f_val] = ga_wrapper(obj_fun, n_vars, lb, ub, params, ...
        n_uavs, n_bombs, ga_opts);
catch
    fprintf('GA不可用，使用fmincon多起点搜索...\n');
    [x_opt, f_val] = multistart_fmincon(obj_fun, n_vars, lb, ub, params, ...
        n_uavs, n_bombs);
end

T_shield_final = -f_val;
fprintf('最优遮蔽时间: %.4f s\n', T_shield_final);

%% 提取结果
result = struct();
result.T_shield = T_shield_final;
result.theta = zeros(1, n_uavs);
result.v = zeros(1, n_uavs);
result.t_releases = zeros(1, n_uavs);
result.delta_ts = zeros(1, n_uavs);
result.release_positions = zeros(n_uavs, 3);
result.det_positions = zeros(n_uavs, 3);
result.uav_ids = 1:n_uavs;

fprintf('\n=== 问题4最优策略 ===\n');
for k = 1:n_uavs
    base = (k-1) * 4;
    th_k = x_opt(base + 1);
    v_k = x_opt(base + 2);
    tr_k = x_opt(base + 3);
    dt_k = x_opt(base + 4);

    result.theta(k) = th_k;
    result.v(k) = v_k;
    result.t_releases(k) = tr_k;
    result.delta_ts(k) = dt_k;

    uav_init = params.uavs(k, :);
    th_rad = deg2rad(th_k);
    result.release_positions(k, :) = uav_traj(uav_init, th_k, v_k, tr_k);
    uav_vel = [v_k * cos(th_rad), v_k * sin(th_rad), 0];
    result.det_positions(k, :) = bomb_traj(result.release_positions(k, :), ...
        uav_vel, tr_k, tr_k + dt_k);

    fprintf('FY%d: θ=%.1f°, v=%.1f m/s, tr=%.2f s, Δt=%.2f s\n', k, ...
        th_k, v_k, tr_k, dt_k);
    fprintf('  投放点: (%.1f, %.1f, %.1f) m\n', result.release_positions(k, :));
    fprintf('  起爆点: (%.1f, %.1f, %.1f) m\n', result.det_positions(k, :));
end

result.x_opt = x_opt;
end

%% 备用函数
function [x_best, f_best] = multistart_fmincon(obj_fun, n_vars, lb, ub, ...
    params, n_uavs, n_bombs)
con_fun = @(x) constraints(x, params, n_uavs, n_bombs);
opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
    'MaxIterations', 200, 'MaxFunctionEvaluations', 5000);

n_starts = 8;
x_best = [];
f_best = inf;

for s = 1:n_starts
    x0 = lb + rand(1, n_vars) .* (ub - lb);
    x0(1:4:end) = rand(1, n_uavs) * 360;
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

if isempty(x_best)
    x_best = (lb + ub) / 2;
    f_best = obj_fun(x_best);
end
end
