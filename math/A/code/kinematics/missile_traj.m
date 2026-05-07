function P = missile_traj(missile_init, t)
% missile_traj — 计算导弹在时刻t的位置
% 输入:
%   missile_init : 导弹初始位置 [x0, y0, z0] (m)
%   t            : 时间，标量或向量 (s)
% 输出:
%   P            : 导弹在t时刻的位置 [x, y, z] (m)，若t为向量则返回矩阵
%
% 导弹以恒定速度300 m/s向假目标(原点)直线飞行

v_mag = 300;  % 导弹速度 (m/s)
dir_vec = -missile_init / norm(missile_init);  % 指向原点的单位方向向量
v = v_mag * dir_vec;

if isscalar(t)
    P = missile_init + v * t;
else
    n = length(t);
    P = zeros(n, 3);
    for i = 1:n
        P(i, :) = missile_init + v * t(i);
    end
end
end
