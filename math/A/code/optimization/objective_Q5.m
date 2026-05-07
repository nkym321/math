function T = objective_Q5(x, params)
% objective_Q5 — 问题5的目标函数
% 5架无人机，每架至多3枚弹，对M1、M2、M3进行干扰
% 最大化对所有3枚导弹的最小遮蔽时间（maximin准则）
%
% 决策变量编码（每架无人机8个变量：θ, v, t_r1, Δt1, t_r2, Δt2, t_r3, Δt3）
% 总计 5×8 = 40 个变量
% 若某枚弹不使用，则 t_r = -1 表示跳过
%
% 输入:
%   x      : 决策变量向量 (1×40)
%   params : 参数字典
% 输出:
%   T      : 负的加权目标值，供GA/优化器最小化

n_uavs = 5;
n_bombs_per_uav = 3;

%% 解析决策变量
theta_list = zeros(1, n_uavs);  % 每架无人机的航向角
v_list = zeros(1, n_uavs);      % 每架无人机的速度
all_t_releases = cell(1, n_uavs); % 每架无人机的投放时刻列表
all_delta_ts = cell(1, n_uavs);   % 每架无人机的起爆延迟列表

for k = 1:n_uavs
    base = (k-1) * 8;
    theta_list(k) = x(base + 1);
    v_list(k) = x(base + 2);

    tr_list = [];
    dt_list = [];
    for b = 1:n_bombs_per_uav
        tr_val = x(base + 2 + 2*b - 1);
        dt_val = x(base + 2 + 2*b);
        if tr_val >= 0  % 有效的投放
            tr_list(end+1) = tr_val;
            dt_list(end+1) = dt_val;
        end
    end
    all_t_releases{k} = tr_list;
    all_delta_ts{k} = dt_list;
end

%% 对每枚导弹分别计算遮蔽时间
T_shield_missiles = zeros(1, 3);

for m = 1:3
    % 收集所有对该导弹有效的干扰弹
    combined_tr = [];
    combined_dt = [];
    combined_uav = [];  % per-bomb: 每枚弹来自哪架UAV

    for k = 1:n_uavs
        if ~isempty(all_t_releases{k})
            n_bombs_k = length(all_t_releases{k});
            combined_tr = [combined_tr, all_t_releases{k}];
            combined_dt = [combined_dt, all_delta_ts{k}];
            % 标记每枚弹的UAV来源
            combined_uav = [combined_uav, repmat(k, 1, n_bombs_k)];
        end
    end

    if isempty(combined_tr)
        T_shield_missiles(m) = 0;
    else
        % per-bomb模式调用：
        %   combined_uav: 每枚弹↦UAV编号
        %   theta_list:  每架UAV的航向角 (按UAV编号索引)
        %   v_list:      每架UAV的速度
        T_shield_missiles(m) = shielding_time(m, combined_uav, ...
            theta_list, v_list, combined_tr, combined_dt, params);
    end
end

%% maximin目标
min_T = min(T_shield_missiles);
sum_T = sum(T_shield_missiles);

% 负值供GA/优化器最小化；α=10优先最大化最小遮蔽时间
alpha = 10;
beta = 1;
T = -(alpha * min_T + beta * sum_T);
end
