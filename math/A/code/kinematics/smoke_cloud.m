function P = smoke_cloud(detonation_pos, t_det, t_eval)
% smoke_cloud — 计算烟幕云团中心在时刻t的位置
% 输入:
%   detonation_pos : 起爆点位置 [x, y, z] (m)
%   t_det          : 起爆时刻 (s)
%   t_eval         : 待求时刻 (s)，必须 >= t_det
% 输出:
%   P              : 烟幕云团中心在t_eval时刻的位置 [x, y, z] (m)
%
% 烟幕云团起爆后以3 m/s的速度匀速下沉

v_sink = 3;  % 下沉速度 (m/s)
dt = t_eval - t_det;

if dt < 0
    error('t_eval 必须 >= t_det');
end

P = detonation_pos;
P(3) = detonation_pos(3) - v_sink * dt;
end
