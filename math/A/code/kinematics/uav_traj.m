function P = uav_traj(uav_init, theta, v, t)
% uav_traj — 计算无人机在时刻t的位置
% 输入:
%   uav_init : 无人机初始位置 [x0, y0, z0] (m)
%   theta    : 航向角，从x轴正方向逆时针测量 (度)，取值范围 [0, 360)
%   v        : 飞行速度 (m/s)，取值范围 [70, 140]
%   t        : 时间，标量或向量 (s)
% 输出:
%   P        : 无人机在t时刻的位置 [x, y, z] (m)
%
% 无人机在接到任务后立即调整航向，以恒定速度沿直线等高度飞行

theta_rad = deg2rad(theta);
vx = v * cos(theta_rad);
vy = v * sin(theta_rad);
vz = 0;  % 等高度飞行

if isscalar(t)
    P = uav_init + [vx, vy, vz] * t;
else
    n = length(t);
    P = zeros(n, 3);
    for i = 1:n
        P(i, :) = uav_init + [vx, vy, vz] * t(i);
    end
end
end
