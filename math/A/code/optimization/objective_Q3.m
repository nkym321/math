function T = objective_Q3(x, params)
% objective_Q3 — 问题3的目标函数（最小化负遮蔽时间，供GA使用）
% FY1投放3枚烟幕干扰弹，对M1进行干扰
% 决策变量 x = [θ, v, t_r1, Δt1, t_r2, Δt2, t_r3, Δt3]
%
% 输入:
%   x      : 决策变量向量 (1×8)
%   params : 参数字典
% 输出:
%   T      : 负的有效遮蔽总时长 (-T_shield)，供GA最小化

theta = x(1);
v_speed = x(2);
t_releases = [x(3), x(5), x(7)];
delta_ts = [x(4), x(6), x(8)];

uav_id = 1;
missile_id = 1;

% 排序投放时刻（确保时间顺序）
[t_releases, sort_idx] = sort(t_releases);
delta_ts = delta_ts(sort_idx);

T_shield = shielding_time(missile_id, uav_id, theta, v_speed, ...
    t_releases, delta_ts, params);

T = -T_shield;  % 返回负值供GA最小化
end
