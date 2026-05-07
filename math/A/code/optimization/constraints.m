function [c, ceq] = constraints(x, params, n_uavs, n_bombs)
% constraints — 非线性约束函数
% 输入:
%   x       : 决策变量向量
%   params  : 参数字典
%   n_uavs  : 无人机数量
%   n_bombs : 每架无人机的弹数
% 输出:
%   c   : 不等式约束 c <= 0
%   ceq : 等式约束 ceq = 0

c = [];
ceq = [];

g = params.g;
v_missile = params.v_missile;
t_impact = norm(params.missiles(1, :)) / v_missile;  % 参考撞击时间

for k = 1:n_uavs
    base = (k-1) * (2 + 2*n_bombs);  % θ, v, then tr,Δt pairs

    if base + 2 > length(x)
        break;
    end

    th = x(base + 1);
    sp = x(base + 2);
    uav_init = params.uavs(k, :);

    for b = 1:n_bombs
        idx_tr = base + 2 + 2*b - 1;
        idx_dt = base + 2 + 2*b;

        if idx_dt > length(x)
            break;
        end

        tr = x(idx_tr);
        dt_val = x(idx_dt);

        if tr < 0
            continue;  % 跳过未使用的弹
        end

        % 约束1: 起爆点z坐标 > 0
        uav_pos = uav_traj(uav_init, th, sp, tr);
        th_rad = deg2rad(th);
        uav_vel = [sp * cos(th_rad), sp * sin(th_rad), 0];
        det_pos = bomb_traj(uav_pos, uav_vel, tr, tr + dt_val);
        c(end+1) = -det_pos(3) + 1;  % z_det >= 1m 安全余量

        % 约束2: 起爆在导弹到达之前
        c(end+1) = (tr + dt_val) - t_impact + 1;

        % 约束3: 投放时刻非负
        c(end+1) = -tr;

        % 约束4: 起爆延迟为正
        c(end+1) = -dt_val + 0.01;
    end

    % 弹间最小间隔约束
    for b = 2:n_bombs
        idx_tr_prev = base + 2 + 2*(b-1) - 1;
        idx_tr_curr = base + 2 + 2*b - 1;

        if idx_tr_curr > length(x)
            break;
        end

        tr_prev = x(idx_tr_prev);
        tr_curr = x(idx_tr_curr);

        if tr_prev >= 0 && tr_curr >= 0
            c(end+1) = params.min_bomb_interval - (tr_curr - tr_prev);
        end
    end
end
end
