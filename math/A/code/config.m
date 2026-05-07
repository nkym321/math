% config.m — 参数初始化与全局配置
% 2025年全国大学生数学建模竞赛 A题：烟幕干扰弹的投放策略

%% 物理常数
params.g = 9.8;              % 重力加速度 (m/s^2)
params.v_missile = 300;      % 导弹飞行速度 (m/s)
params.v_smoke_sink = 3;     % 烟幕云团下沉速度 (m/s)
params.R_smoke = 10;         % 烟幕云团有效半径 (m)
params.T_smoke_eff = 20;     % 烟幕云团有效持续时间 (s)
params.v_uav_min = 70;       % 无人机最小飞行速度 (m/s)
params.v_uav_max = 140;      % 无人机最大飞行速度 (m/s)
params.min_bomb_interval = 1;% 连续投放两枚烟幕干扰弹的最小间隔 (s)
params.R_target = 7;         % 目标圆柱半径 (m)
params.H_target = 10;        % 目标圆柱高度 (m)

%% 初始位置信息
% 导弹初始位置 [x, y, z] (m)
params.missiles = [20000, 0, 2000;      % M1
                   19000, 600, 2100;    % M2
                   18000, -600, 1900];  % M3

% 无人机初始位置 [x, y, z] (m)
params.uavs = [17800, 0, 1800;      % FY1
               12000, 1400, 1400;   % FY2
               6000, -3000, 700;    % FY3
               11000, 2000, 1800;   % FY4
               13000, -2000, 1300]; % FY5

% 假目标在原点 (0,0,0)，真目标圆柱底面圆心
params.target_center = [0, 200, 0];  % 真目标底面圆心 (m)
params.decoy_center = [0, 0, 0];     % 假目标位置 (原点)

%% 数值离散化参数
params.N_target_pts = 80;      % 目标圆柱离散采样点数
params.dt = 0.01;              % 仿真时间步长 (s)

%% 优化算法参数
params.ga_population = 200;    % 遗传算法种群大小
params.ga_generations = 300;   % 遗传算法最大代数
params.ga_crossover = 0.8;     % 交叉概率
params.ga_mutation = 0.1;      % 变异概率
params.ga_elite = 0.05;        % 精英比例
params.ga_stall_limit = 50;    % 停滞代数限制

%% 输出路径
params.output_dir = '../output/';
params.figure_dir = '../output/figures/';

fprintf('配置加载完成。\n');
