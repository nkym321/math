function P = bomb_traj(release_pos, uav_vel, t_release, t_eval)
% bomb_traj — 计算烟幕干扰弹在自由落体阶段的位置
% 输入:
%   release_pos : 投放点位置 [x, y, z] (m)
%   uav_vel     : 投放瞬间无人机的速度向量 [vx, vy, 0] (m/s)
%   t_release   : 投放时刻 (s)
%   t_eval      : 待求时刻 (s)，必须 >= t_release
% 输出:
%   P           : 干扰弹在t_eval时刻的位置 [x, y, z] (m)
%
% 干扰弹脱离无人机后，继承无人机水平速度，在重力作用下运动

g = 9.8;
dt = t_eval - t_release;

if dt < 0
    error('t_eval 必须 >= t_release');
end

P = zeros(1, 3);
P(1) = release_pos(1) + uav_vel(1) * dt;
P(2) = release_pos(2) + uav_vel(2) * dt;
P(3) = release_pos(3) - 0.5 * g * dt^2;
end
