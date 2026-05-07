function plot_shielding(results, params)
% plot_shielding — 绘制遮蔽效果可视化
% 热力图：展示不同参数组合下的遮蔽时间
%
% 输入:
%   results : 结果结构体（可包含多个问题的结果）
%   params  : 参数字典

figure('Position', [100, 100, 1000, 400]);

%% 子图1：遮蔽时间随起爆延迟和投放时刻的变化（Q2参数扫描）
subplot(1, 2, 1);

% 固定航向和速度（使用Q2的最优结果或默认值）
theta_fixed = 180;  % 朝向假目标
v_fixed = 120;
uav_id = 1;
missile_id = 1;

% 扫描投放时刻和起爆延迟
tr_range = 0:1:12;
dt_range = 0.5:0.5:15;
T_matrix = zeros(length(dt_range), length(tr_range));

for i = 1:length(dt_range)
    for j = 1:length(tr_range)
        T_matrix(i, j) = shielding_time(missile_id, uav_id, ...
            theta_fixed, v_fixed, tr_range(j), dt_range(i), params);
    end
end

imagesc(tr_range, dt_range, T_matrix);
colorbar;
xlabel('投放时刻 t_r (s)', 'FontSize', 11);
ylabel('起爆延迟 Δt (s)', 'FontSize', 11);
title(sprintf('遮蔽时间热力图 (θ=%.0f°, v=%.0f m/s)', theta_fixed, v_fixed), ...
    'FontSize', 12);
colormap(jet);
set(gca, 'YDir', 'normal');

%% 子图2：各问题结果对比
subplot(1, 2, 2);

% 柱状图
if isfield(results, 'Q1')
    T_list = [results.Q1.T_shield];
    labels = {'Q1'};
    if isfield(results, 'Q2')
        T_list = [T_list, results.Q2.T_shield];
        labels{end+1} = 'Q2';
    end
    if isfield(results, 'Q3')
        T_list = [T_list, results.Q3.T_shield];
        labels{end+1} = 'Q3';
    end
    if isfield(results, 'Q4')
        T_list = [T_list, results.Q4.T_shield];
        labels{end+1} = 'Q4';
    end
    if isfield(results, 'Q5')
        T_list = [T_list, results.Q5.T_shield];
        labels{end+1} = 'Q5';
    end

    bar(T_list);
    set(gca, 'XTickLabel', labels, 'FontSize', 11);
    ylabel('有效遮蔽时长 (s)', 'FontSize', 11);
    title('各问题最优遮蔽时长对比', 'FontSize', 12);
    grid on;
end

%% 保存
save_filename = fullfile(params.figure_dir, 'shielding_analysis.png');
exportgraphics(gcf, save_filename, 'Resolution', 200);
fprintf('遮蔽分析图已保存至: %s\n', save_filename);
end
