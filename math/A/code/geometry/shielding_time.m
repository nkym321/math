function T_shield = shielding_time(missile_id, uav_id, theta, v, t_releases, delta_ts, params)
% shielding_time — 计算烟幕干扰弹对来袭导弹的有效遮蔽总时长
% 输入:
%   missile_id  : 导弹编号 (1,2,3)
%   uav_id      : 方案A(per-UAV): 无人机编号向量 [1] 或 [1,2,3]
%                 方案B(per-bomb): 每枚弹对应的UAV编号，长度=t_releases
%   theta       : 方案A: 各无人机的航向角向量 (度), 长度=length(uav_id)
%                 方案B: 各无人机的航向角, 按uav_id中的值索引
%   v           : 各无人机的飞行速度向量 (m/s), 同theta的格式
%   t_releases  : 各干扰弹的投放时刻向量 (s)
%   delta_ts    : 各干扰弹的起爆延迟时间向量 (s)
%   params      : 参数字典
% 输出:
%   T_shield    : 有效遮蔽总时长 (s)
%
% 自动检测模式：若 length(uav_id) == length(t_releases)，使用per-bomb模式；
%              否则使用per-UAV模式（等分炸弹）。

%% 初始化
missile_init = params.missiles(missile_id, :);
R_smoke = params.R_smoke;
T_eff = params.T_smoke_eff;
dt = params.dt;

% 生成目标采样点
target_pts = target_points(params);
n_target = size(target_pts, 1);

% 计算导弹到达假目标(原点)的时间
t_impact = norm(missile_init) / params.v_missile;

% 生成时间序列
t_vec = 0:dt:t_impact;
n_steps = length(t_vec);

% 如果没有任何干扰弹，直接返回0
if isempty(t_releases)
    T_shield = 0;
    return;
end

%% 预计算导弹在每个时间步的位置
missile_positions = zeros(n_steps, 3);
for i = 1:n_steps
    missile_positions(i, :) = missile_traj(missile_init, t_vec(i));
end

%% 判断模式：per-UAV 还是 per-bomb
% per-bomb模式：uav_id长度==弹数，且有重复UAV编号（不同弹来自同一UAV）
% per-UAV模式：uav_id是各不相同的UAV编号，弹数被均分
n_bombs = length(t_releases);
if length(uav_id) == n_bombs && length(unique(uav_id)) < length(uav_id)
    per_bomb_mode = true;  % 有重复UAV ID → 每弹单独指定来源
else
    per_bomb_mode = false; % 弹数均分给各UAV
end

%% 计算每枚干扰弹的关键事件时间
t_det = zeros(1, n_bombs);
t_expire = zeros(1, n_bombs);
det_pos = zeros(n_bombs, 3);

for bomb_idx = 1:n_bombs
    if per_bomb_mode
        uid = uav_id(bomb_idx);
        % theta和v按UAV ID索引
        th = theta(uid);
        sp = v(uid);
    else
        % 确定当前弹属于哪个UAV
        bombs_per_uav = n_bombs / length(uav_id);
        uav_idx = ceil(bomb_idx / bombs_per_uav);
        uid = uav_id(uav_idx);
        th = theta(uav_idx);
        sp = v(uav_idx);
    end

    uav_init = params.uavs(uid, :);

    % 投放时刻无人机位置
    uav_pos_release = uav_traj(uav_init, th, sp, t_releases(bomb_idx));

    % 无人机速度向量
    theta_rad = deg2rad(th);
    uav_vel = [sp * cos(theta_rad), sp * sin(theta_rad), 0];

    % 起爆位置
    det_pos(bomb_idx, :) = bomb_traj(uav_pos_release, uav_vel, ...
        t_releases(bomb_idx), t_releases(bomb_idx) + delta_ts(bomb_idx));

    % 起爆时刻和失效时刻
    t_det(bomb_idx) = t_releases(bomb_idx) + delta_ts(bomb_idx);
    t_expire(bomb_idx) = t_det(bomb_idx) + T_eff;
end

%% 逐时间步检查遮蔽情况
shielded = false(1, n_steps);

for i = 1:n_steps
    t_now = t_vec(i);
    M_pos = missile_positions(i, :);

    % 检查每枚干扰弹是否处于有效期内
    for b = 1:n_bombs
        if t_now < t_det(b) || t_now > t_expire(b)
            continue;
        end

        % 烟幕云团当前中心位置
        S_center = smoke_cloud(det_pos(b, :), t_det(b), t_now);

        % 检查所有目标采样点的视线是否被遮挡
        all_blocked = true;
        for j = 1:n_target
            if ~los_blocked(M_pos, target_pts(j, :), S_center, R_smoke)
                all_blocked = false;
                break;
            end
        end

        if all_blocked
            shielded(i) = true;
            break;
        end
    end
end

%% 累积有效遮蔽时间
T_shield = sum(shielded) * dt;
end
