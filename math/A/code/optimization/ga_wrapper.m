function [x_opt, f_opt] = ga_wrapper(obj_fun, n_vars, lb, ub, params, ...
    n_uavs, n_bombs, ga_opts)
% ga_wrapper — 遗传算法优化包装器
% 输入:
%   obj_fun  : 目标函数句柄
%   n_vars   : 决策变量数量
%   lb       : 变量下界向量
%   ub       : 变量上界向量
%   params   : 参数字典
%   n_uavs   : 无人机数量（用于约束）
%   n_bombs  : 每架无人机弹数（用于约束）
%   ga_opts  : 可选，GA参数字典，可覆盖默认值
% 输出:
%   x_opt    : 最优决策变量
%   f_opt    : 最优目标函数值
%
% 使用 MATLAB Global Optimization Toolbox 的 ga 函数

%% GA参数设置
if nargin < 8
    ga_opts = struct();
end

pop_size = get_field(ga_opts, 'PopulationSize', params.ga_population);
max_gen = get_field(ga_opts, 'MaxGenerations', params.ga_generations);
cross_frac = get_field(ga_opts, 'CrossoverFraction', params.ga_crossover);
mut_rate = get_field(ga_opts, 'MutationRate', params.ga_mutation);
elite_count = max(1, round(pop_size * params.ga_elite));
stall_limit = get_field(ga_opts, 'StallGenLimit', params.ga_stall_limit);

%% 创建约束函数句柄
con_fun = @(x) constraints(x, params, n_uavs, n_bombs);

%% GA选项
options = optimoptions('ga', ...
    'PopulationSize', pop_size, ...
    'MaxGenerations', max_gen, ...
    'CrossoverFraction', cross_frac, ...
    'EliteCount', elite_count, ...
    'FunctionTolerance', 1e-4, ...
    'ConstraintTolerance', 1e-3, ...
    'MaxStallGenerations', stall_limit, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf, @gaplotstopping}, ...
    'UseParallel', false);

% 设置变异函数（使用自适应可行变异）
options.MutationFcn = {@mutationadaptfeasible, mut_rate};

% 设置交叉函数
options.CrossoverFcn = @crossoverscattered;

%% 运行GA
fprintf('启动遗传算法...\n');
fprintf('  变量数: %d, 种群: %d, 最大代数: %d\n', n_vars, pop_size, max_gen);

rng(42);  % 固定随机种子以保证可重复性

[x_opt, f_opt, exitflag, output] = ga(obj_fun, n_vars, [], [], [], [], ...
    lb, ub, con_fun, options);

fprintf('GA完成: exitflag=%d, 代数=%d\n', exitflag, output.generations);
fprintf('最优目标值: %.4f\n', f_opt);
end

%% 辅助函数
function val = get_field(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
