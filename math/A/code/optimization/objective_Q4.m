function T = objective_Q4(x, params)
% objective_Q4 — 问题4的目标函数（最小化负遮蔽时间，供GA使用）
% FY1、FY2、FY3各投放1枚烟幕干扰弹，对M1进行干扰
% 决策变量 x = [θ1, v1, t_r1, Δt1, θ2, v2, t_r2, Δt2, θ3, v3, t_r3, Δt3]
%
% 输入:
%   x      : 决策变量向量 (1×12)
%   params : 参数字典
% 输出:
%   T      : 负的有效遮蔽总时长 (-T_shield)，供GA最小化

theta = [x(1), x(5), x(9)];
v_speed = [x(2), x(6), x(10)];
t_releases = [x(3), x(7), x(11)];
delta_ts = [x(4), x(8), x(12)];

uav_id = [1, 2, 3];
missile_id = 1;

T_shield = shielding_time(missile_id, uav_id, theta, v_speed, ...
    t_releases, delta_ts, params);

T = -T_shield;
end
