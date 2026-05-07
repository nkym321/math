function plot_trajectories(results, params)
% plot_trajectories — 绘制3D轨迹总览图
% 显示：导弹轨迹、无人机轨迹、投放点、起爆点、真/假目标
%
% 输入:
%   results : 包含各问题结果的结构体（或cell array）
%   params  : 参数字典

figure('Position', [100, 100, 1200, 800]);
hold on; grid on; box on;
view(45, 20);

%% 绘制真目标和假目标
% 假目标（原点）
scatter3(0, 0, 0, 80, 'r', 'filled', 'DisplayName', '假目标(原点)');
% 真目标圆柱
draw_cylinder(params.target_center, params.R_target, params.H_target, ...
    [0.2, 0.8, 0.2], 0.5, '真目标');

%% 绘制导弹轨迹
colors_m = {'r', 'm', 'b'};
for m = 1:3
    missile_init = params.missiles(m, :);
    t_impact = norm(missile_init) / params.v_missile;
    t_vec = linspace(0, t_impact, 100);
    traj = zeros(length(t_vec), 3);
    for i = 1:length(t_vec)
        traj(i, :) = missile_traj(missile_init, t_vec(i));
    end
    plot3(traj(:,1), traj(:,2), traj(:,3), '-', 'Color', colors_m{m}, ...
        'LineWidth', 1.5, 'DisplayName', sprintf('M%d轨迹', m));
    % 导弹初始位置标注
    scatter3(missile_init(1), missile_init(2), missile_init(3), 60, ...
        colors_m{m}, 'filled', '^', 'DisplayName', sprintf('M%d初始', m));
end

%% 绘制无人机初始位置
colors_uav = lines(5);
for k = 1:5
    uav_init = params.uavs(k, :);
    scatter3(uav_init(1), uav_init(2), uav_init(3), 60, ...
        colors_uav(k,:), 'filled', 's', ...
        'DisplayName', sprintf('FY%d初始', k));
end

%% 标注
xlabel('X (北) [m]', 'FontSize', 12);
ylabel('Y (东) [m]', 'FontSize', 12);
zlabel('Z (高度) [m]', 'FontSize', 12);
title('烟幕干扰弹投放策略 — 3D轨迹总览', 'FontSize', 14);
legend('Location', 'bestoutside', 'FontSize', 8);
axis equal;

%% 保存
save_filename = fullfile(params.figure_dir, 'trajectories_overview.png');
exportgraphics(gcf, save_filename, 'Resolution', 200);
fprintf('轨迹图已保存至: %s\n', save_filename);
end

%% 辅助函数：绘制圆柱
function draw_cylinder(center, R, H, color, alpha_val, label)
    [X, Y, Z] = cylinder(R, 40);
    Z = Z * H + center(3);
    X = X + center(1);
    Y = Y + center(2);
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none', ...
        'FaceAlpha', alpha_val, 'DisplayName', label);
    % 顶面
    fill3(X(2,:), Y(2,:), Z(2,:), color, 'FaceAlpha', alpha_val, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
    % 底面
    fill3(X(1,:), Y(1,:), Z(1,:), color, 'FaceAlpha', alpha_val, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
end
