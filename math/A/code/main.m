% main.m — 主控脚本
% 2025年全国大学生数学建模竞赛 A题：烟幕干扰弹的投放策略
% 依次求解问题1~5，生成所有结果文件和可视化图表
%
% 使用方法：在MATLAB中运行此脚本
%   确保当前目录为 code/ 或其父目录
%   确保已添加所有子目录到路径

clear; clc; close all;

%% 路径设置
% 添加所有子目录到MATLAB搜索路径
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
addpath(genpath(script_dir));

%% 加载参数配置
fprintf('========================================\n');
fprintf('  2025 CUMCM A题：烟幕干扰弹投放策略\n');
fprintf('========================================\n\n');

config;
params.output_dir = fullfile(script_dir, '..', 'output');
params.figure_dir = fullfile(params.output_dir, 'figures');

% 确保输出目录存在
if ~exist(params.output_dir, 'dir')
    mkdir(params.output_dir);
end
if ~exist(params.figure_dir, 'dir')
    mkdir(params.figure_dir);
end

%% 存储所有结果的容器
all_results = struct();

% %% 问题1：确定性计算
% fprintf('\n########################################\n');
% fprintf('#  问题1：确定性计算\n');
% fprintf('########################################\n');
% try
%     all_results.Q1 = solve_Q1(params);
% catch ME
%     fprintf('问题1求解出错: %s\n', ME.message);
%     all_results.Q1 = struct('T_shield', 0);
% end
% 
% %% 问题2：单枚弹最优策略
% fprintf('\n########################################\n');
% fprintf('#  问题2：单枚弹最优投放策略\n');
% fprintf('########################################\n');
% try
%     all_results.Q2 = solve_Q2(params);
% catch ME
%     fprintf('问题2求解出错: %s\n', ME.message);
%     all_results.Q2 = struct('T_shield', 0);
% end
% 
% %% 问题3：三枚弹策略
% fprintf('\n########################################\n');
% fprintf('#  问题3：三枚弹投放策略\n');
% fprintf('########################################\n');
% try
%     all_results.Q3 = solve_Q3(params);
%     write_result1(all_results.Q3, fullfile(params.output_dir, 'result1.xlsx'));
% catch ME
%     fprintf('问题3求解出错: %s\n', ME.message);
%     all_results.Q3 = struct('T_shield', 0);
% end
% 
% %% 问题4：三架无人机协同
% fprintf('\n########################################\n');
% fprintf('#  问题4：三架无人机协同投放\n');
% fprintf('########################################\n');
% try
%     all_results.Q4 = solve_Q4(params);
%     write_result2(all_results.Q4, fullfile(params.output_dir, 'result2.xlsx'));
% catch ME
%     fprintf('问题4求解出错: %s\n', ME.message);
%     all_results.Q4 = struct('T_shield', 0);
% end

%% 问题5：全规模问题
fprintf('\n########################################\n');
fprintf('#  问题5：五架无人机对三枚导弹\n');
fprintf('########################################\n');
try
    all_results.Q5 = solve_Q5(params);
    write_result3(all_results.Q5, fullfile(params.output_dir, 'result3.xlsx'));
catch ME
    fprintf('问题5求解出错: %s\n', ME.message);
    all_results.Q5 = struct('T_shield', 0);
end

%% 结果汇总
fprintf('\n\n========================================\n');
fprintf('  结果汇总\n');
fprintf('========================================\n');

q_names = {'Q1', 'Q2', 'Q3', 'Q4', 'Q5'};
for i = 1:5
    qn = q_names{i};
    if isfield(all_results, qn) && isfield(all_results.(qn), 'T_shield')
        if i == 5 && isfield(all_results.Q5, 'T_by_missile')
            fprintf('问题%d: 总遮蔽时长 = %.4f s (M1:%.4f, M2:%.4f, M3:%.4f)\n', ...
                i, all_results.Q5.T_shield, all_results.Q5.T_by_missile);
        else
            fprintf('问题%d: 有效遮蔽时长 = %.4f s\n', i, all_results.(qn).T_shield);
        end
    else
        fprintf('问题%d: 未求解\n', i);
    end
end

%% 生成可视化图表
fprintf('\n========================================\n');
fprintf('  生成可视化图表\n');
fprintf('========================================\n');

try
    plot_trajectories(all_results, params);
catch ME
    fprintf('轨迹图生成失败: %s\n', ME.message);
end

try
    plot_shielding(all_results, params);
catch ME
    fprintf('遮蔽分析图生成失败: %s\n', ME.message);
end

try
    plot_convergence([], params);
catch ME
    fprintf('收敛曲线图生成失败: %s\n', ME.message);
end

%% 保存MATLAB结果数据
fprintf('\n保存结果数据...\n');
save(fullfile(params.output_dir, 'all_results.mat'), 'all_results', 'params');
fprintf('结果数据已保存至 all_results.mat\n');

fprintf('\n========================================\n');
fprintf('  全部任务完成！\n');
fprintf('  输出目录: %s\n', params.output_dir);
fprintf('========================================\n');
