function result = solve_Q1(params)
% solve_Q1 — 问题1：确定性计算有效遮蔽时长
% 条件：FY1以120 m/s朝向假目标飞行，受领任务1.5s后投放1枚干扰弹，
%       间隔3.6s后起爆。计算对M1的有效遮蔽时长。
% 输入:
%   params : 参数字典
% 输出:
%   result : 结构体，包含 T_shield 及各中间量

fprintf('\n========== 问题1 ==========\n');

%% 已知参数
uav_id = 1;          % FY1
missile_id = 1;      % M1
v = 120;             % 飞行速度 (m/s)

% 航向：FY1朝向假目标(原点)
uav_init = params.uavs(uav_id, :);
% 从FY1位置指向原点的方向（仅xy平面）
dir_to_origin = [0, 0] - uav_init(1:2);
theta = rad2deg(atan2(dir_to_origin(2), dir_to_origin(1)));
fprintf('航向角 θ = %.2f° (朝假目标方向)\n', theta);

t_release = 1.5;     % 受领任务后投放时间 (s)
delta_t = 3.6;       % 起爆延迟 (s)

%% 投放点位置
uav_pos_release = uav_traj(uav_init, theta, v, t_release);
fprintf('投放点位置: (%.2f, %.2f, %.2f) m\n', uav_pos_release);

%% 起爆点位置
theta_rad = deg2rad(theta);
uav_vel = [v * cos(theta_rad), v * sin(theta_rad), 0];
det_pos = bomb_traj(uav_pos_release, uav_vel, t_release, t_release + delta_t);
fprintf('起爆点位置: (%.2f, %.2f, %.2f) m\n', det_pos);

%% 计算有效遮蔽时长
T_shield = shielding_time(missile_id, uav_id, theta, v, t_release, delta_t, params);
fprintf('有效遮蔽时长: %.4f s\n', T_shield);

%% 返回结果
result = struct();
result.T_shield = T_shield;
result.theta = theta;
result.v = v;
result.t_release = t_release;
result.delta_t = delta_t;
result.uav_pos_release = uav_pos_release;
result.det_pos = det_pos;
result.t_det = t_release + delta_t;
result.t_expire = t_release + delta_t + params.T_smoke_eff;
end
