function blocked = los_blocked(missile_pos, target_pt, smoke_center, R_smoke)
% los_blocked — 判断从导弹到目标点的视线是否被烟幕球体遮挡
% 输入:
%   missile_pos  : 导弹位置 [x, y, z] (m)
%   target_pt    : 目标表面点 [x, y, z] (m)
%   smoke_center : 烟幕云团球心位置 [x, y, z] (m)
%   R_smoke      : 烟幕球体有效半径 (m)
% 输出:
%   blocked      : 逻辑值，true表示视线被遮挡
%
% 算法：计算导弹到目标点的线段上距离球心最近的点，
%       若该距离 <= 球体半径，则视线被遮挡。

MT = target_pt - missile_pos;          % 从导弹指向目标点的向量
SM = smoke_center - missile_pos;       % 从导弹指向球心的向量
MT_norm2 = dot(MT, MT);

if MT_norm2 < 1e-10
    % 导弹已到达目标点
    blocked = false;
    return;
end

% 球心到直线的投影参数 λ*
lambda = dot(SM, MT) / MT_norm2;

% 截断到线段 [0, 1]
lambda_c = max(0, min(1, lambda));

% 线段上距离球心最近的点
closest_pt = missile_pos + lambda_c * MT;

% 判断最近距离是否小于等于球体半径
dist2 = sum((closest_pt - smoke_center).^2);
blocked = (dist2 <= R_smoke^2);
end
