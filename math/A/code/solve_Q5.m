function result = solve_Q5(params)
% solve_Q5 — 问题5：5架UAV（每架≤3弹）对M1、M2、M3的协同干扰
%
% 几何现实（重要）：
%   导弹飞行速度300m/s > UAV最大速度140m/s。
%   UAV无法从后方超越导弹。只有初始位置比导弹更靠近目标的UAV
%   （x坐标更小的UAV）才能在导弹和目标之间放置烟幕。
%   - M1在x=20000：所有UAV(x≤17800)都在它前面 → 都可拦截
%   - M2在x=19000：FY2(x=12000),FY3(6000),FY4(11000),FY5(13000)在它前面
%   - M3在x=18000：只有FY2/3/4/5在它前面，FY1(x=17800)几乎平行
%
% 策略：
%   阶段1: 每UAV×每导弹几何引导搜索（非纯随机）
%   阶段2: 贪心分配，确保每枚导弹至少1架UAV
%   阶段3: 追加额外弹
%   阶段4: 全局汇总

fprintf('\n========== 问题5 ==========\n');

n_uavs = 5;
n_missiles = 3;
n_bombs_max = 3;

%% 导弹撞击时间
t_impact = zeros(1, n_missiles);
for m = 1:n_missiles
    t_impact(m) = norm(params.missiles(m, :)) / params.v_missile;
end
fprintf('导弹飞行时间: M1=%.1fs, M2=%.1fs, M3=%.1fs\n\n', t_impact);

%% ================================================================
%  阶段1：几何引导搜索
%  对每个(UAV,导弹)组合：
%    a) 沿导弹轨迹采样N个时刻
%    b) 在每个时刻，计算导弹→真目标的LOS
%    c) 测试UAV能否在LOS附近放置烟幕
%    d) 补充纯随机搜索作为探索
%  ================================================================
fprintf('=== 阶段1: UAV×导弹 几何引导搜索 ===\n');

best_T_km   = zeros(n_uavs, n_missiles);
best_th_km  = zeros(n_uavs, n_missiles);
best_v_km   = zeros(n_uavs, n_missiles);
best_tr_km  = zeros(n_uavs, n_missiles);
best_dt_km  = zeros(n_uavs, n_missiles);

% 真目标中心（用于LOS计算）
T_center = [params.target_center(1:2), params.H_target/2];  % (0, 200, 5)

rng(42);

for k = 1:n_uavs
    uav_init = params.uavs(k, :);
    for m = 1:n_missiles
        t_max = t_impact(m);
        missile_init = params.missiles(m, :);
        best_T = -inf;
        best_sol = [];

        % ---- 几何引导部分：沿导弹轨迹采样 ----
        n_samples = 50;  % 导弹轨迹采样点
        for s = 0:n_samples
            t_m = (s / n_samples) * t_max * 0.9;  % 导弹飞行到90%位置
            M_pos = missile_traj(missile_init, t_m);

            % 导弹→目标LOS方向(仅xy平面)
            los_dir_xy = T_center(1:2) - M_pos(1:2);
            los_dist = norm(los_dir_xy);
            if los_dist < 1, continue; end
            los_dir_xy = los_dir_xy / los_dist;

            % 在LOS上取几个候选烟幕位置（导弹与目标之间）
            for lambda = [0.2, 0.4, 0.6, 0.8]
                smoke_target_xy = M_pos(1:2) + lambda * los_dir_xy * los_dist;

                % UAV飞向该位置需要的航向
                to_target = smoke_target_xy - uav_init(1:2);
                dist_to_target = norm(to_target);
                if dist_to_target < 1, continue; end

                % 尝试不同速度
                for v_test = [80, 100, 120, 140]
                    % 到达目标xy位置的时间
                    t_arrive = dist_to_target / v_test;
                    if t_arrive > t_max * 0.95, continue; end

                    % 尝试在到达前/后投放
                    for t_offset = [-5, -2, 0, 2, 5]
                        tr = t_arrive + t_offset;
                        if tr < 0 || tr > t_max * 0.9, continue; end

                        % UAV在tr时刻的位置
                        th_test = rad2deg(atan2(to_target(2), to_target(1)));
                        uav_at_tr = uav_traj(uav_init, th_test, v_test, tr);

                        % 尝试不同起爆延迟
                        for dt_test = [2, 4, 6, 8, 10, 12]
                            td = tr + dt_test;
                            if td >= t_max, continue; end

                            % 起爆位置
                            th_rad = deg2rad(th_test);
                            uav_vel = [v_test*cos(th_rad), v_test*sin(th_rad), 0];
                            det_pos = bomb_traj(uav_at_tr, uav_vel, tr, td);
                            if det_pos(3) <= 0, continue; end

                            T = shielding_time(m, [k], th_test, v_test, ...
                                [tr], [dt_test], params);
                            if T > best_T
                                best_T = T;
                                best_sol = [th_test, v_test, tr, dt_test];
                            end
                        end
                    end
                end
            end
        end

        % ---- 补充随机搜索 ----
        n_random = 3000;
        for trial = 1:n_random
            th  = rand * 360;
            sp  = 70 + rand * 70;
            tr1 = rand * t_max * 0.85;
            dt1 = 1 + rand * 15;

            td1 = tr1 + dt1;
            if td1 >= t_max, continue; end

            uav_at_rel = uav_traj(uav_init, th, sp, tr1);
            th_rad = deg2rad(th);
            uav_vel = [sp*cos(th_rad), sp*sin(th_rad), 0];
            det_pos = bomb_traj(uav_at_rel, uav_vel, tr1, td1);
            if det_pos(3) <= 0, continue; end

            T = shielding_time(m, [k], th, sp, [tr1], [dt1], params);
            if T > best_T
                best_T = T;
                best_sol = [th, sp, tr1, dt1];
            end
        end

        if ~isempty(best_sol)
            best_T_km(k,m)  = best_T;
            best_th_km(k,m) = best_sol(1);
            best_v_km(k,m)  = best_sol(2);
            best_tr_km(k,m) = best_sol(3);
            best_dt_km(k,m) = best_sol(4);
        end
    end
    fprintf('FY%d: M1=%.2fs  M2=%.2fs  M3=%.2fs\n', k, ...
        best_T_km(k,1), best_T_km(k,2), best_T_km(k,3));
end

%% ================================================================
%  阶段2：贪心分配
%  ================================================================
fprintf('\n=== 阶段2: UAV→导弹 分配 ===\n');

% 收集所有正评分的(UAV, missile)对
pairs = [];
for k = 1:n_uavs
    for m = 1:n_missiles
        if best_T_km(k,m) > 0
            pairs(end+1, :) = [k, m, best_T_km(k,m)];
        end
    end
end

assigned = zeros(1, n_uavs);
missile_has_uav = false(1, n_missiles);

if ~isempty(pairs)
    pairs = sortrows(pairs, -3);  % 按评分降序

    % 第一轮：每架UAV选评分最高的导弹
    for i = 1:size(pairs, 1)
        k = pairs(i, 1);
        m = pairs(i, 2);
        if assigned(k) == 0
            assigned(k) = m;
            missile_has_uav(m) = true;
        end
    end

    % 第二轮：填补未覆盖的导弹
    for iter = 1:10
        uncovered = find(~missile_has_uav);
        if isempty(uncovered), break; end

        for m = uncovered
            [best_T_for_m, best_k] = max(best_T_km(:, m));
            if best_T_for_m <= 0, continue; end
            old_m = assigned(best_k);
            assigned(best_k) = m;
            % 重新计算覆盖
            missile_has_uav(:) = false;
            for kk = 1:n_uavs
                if assigned(kk) > 0
                    missile_has_uav(assigned(kk)) = true;
                end
            end
            fprintf('  FY%d: M%d→M%d (评分%.2fs)\n', best_k, old_m, m, best_T_for_m);
        end
    end
end

% 对未被分配的UAV，给默认参数（飞向最近的导弹）
for k = 1:n_uavs
    if assigned(k) == 0
        [~, m_default] = min(vecnorm(params.uavs(k,:) - params.missiles, 2, 2));
        assigned(k) = m_default;
        % 用随机默认参数
        uav_init = params.uavs(k, :);
        mid_pt = params.missiles(m_default, :) * 0.4;
        d = mid_pt(1:2) - uav_init(1:2);
        best_th_km(k, m_default) = rad2deg(atan2(d(2), d(1)));
        best_v_km(k, m_default)  = 120;
        best_tr_km(k, m_default) = max(1, norm(d) / 120 - 3);
        best_dt_km(k, m_default) = 5;
        fprintf('FY%d: 无可行解, 默认→M%d\n', k, m_default);
    end
end

for k = 1:n_uavs
    fprintf('FY%d → M%d\n', k, assigned(k));
end

%% ================================================================
%  阶段3：追加额外弹
%  ================================================================
fprintf('\n=== 阶段3: 追加额外弹 ===\n');

theta_opt = zeros(1, n_uavs);
v_opt     = zeros(1, n_uavs);
tr_opt    = cell(1, n_uavs);
dt_opt    = cell(1, n_uavs);

for k = 1:n_uavs
    m = assigned(k);
    uav_init = params.uavs(k, :);

    theta_opt(k) = best_th_km(k, m);
    v_opt(k)     = best_v_km(k, m);
    tr_opt{k}    = [best_tr_km(k, m)];
    dt_opt{k}    = [best_dt_km(k, m)];

    th = theta_opt(k); sp = v_opt(k);
    th_rad = deg2rad(th);
    uav_vel = [sp*cos(th_rad), sp*sin(th_rad), 0];

    % 第2枚弹
    td1 = tr_opt{k}(1) + dt_opt{k}(1);
    best_sum2 = -inf; best_tr2 = []; best_dt2 = [];
    for tr2 = td1 + (6:2:16)
        if tr2 < tr_opt{k}(1) + 1, continue; end
        if tr2 > t_impact(m) * 0.8, continue; end
        for dt2 = [3, 5, 7]
            td2 = tr2 + dt2;
            if td2 >= t_impact(m), continue; end
            uav_at = uav_traj(uav_init, th, sp, tr2);
            det_pos = bomb_traj(uav_at, uav_vel, tr2, td2);
            if det_pos(3) <= 0, continue; end
            all_tr = [tr_opt{k}, tr2]; all_dt = [dt_opt{k}, dt2];
            sumT = 0;
            for mm = 1:n_missiles
                sumT = sumT + shielding_time(mm, [k], th, sp, all_tr, all_dt, params);
            end
            if sumT > best_sum2
                best_sum2 = sumT; best_tr2 = tr2; best_dt2 = dt2;
            end
        end
    end
    if ~isempty(best_tr2)
        tr_opt{k}(end+1) = best_tr2; dt_opt{k}(end+1) = best_dt2;
        fprintf('FY%d→M%d: +弹2 tr=%.1fs Δt=%.1fs\n', k, m, best_tr2, best_dt2);
    end

    % 第3枚弹
    if length(tr_opt{k}) >= 2
        td2 = tr_opt{k}(2) + dt_opt{k}(2);
        best_sum3 = -inf; best_tr3 = []; best_dt3 = [];
        for tr3 = td2 + (4:2:12)
            if tr3 < tr_opt{k}(2) + 1, continue; end
            if tr3 > t_impact(m) * 0.8, continue; end
            for dt3 = [3, 5, 7]
                td3 = tr3 + dt3;
                if td3 >= t_impact(m), continue; end
                uav_at = uav_traj(uav_init, th, sp, tr3);
                det_pos = bomb_traj(uav_at, uav_vel, tr3, td3);
                if det_pos(3) <= 0, continue; end
                all_tr = [tr_opt{k}, tr3]; all_dt = [dt_opt{k}, dt3];
                sumT = 0;
                for mm = 1:n_missiles
                    sumT = sumT + shielding_time(mm, [k], th, sp, all_tr, all_dt, params);
                end
                if sumT > best_sum3
                    best_sum3 = sumT; best_tr3 = tr3; best_dt3 = dt3;
                end
            end
        end
        if ~isempty(best_tr3)
            tr_opt{k}(end+1) = best_tr3; dt_opt{k}(end+1) = best_dt3;
            fprintf('FY%d→M%d: +弹3 tr=%.1fs Δt=%.1fs\n', k, m, best_tr3, best_dt3);
        end
    end
end

%% ================================================================
%  阶段4：全局汇总
%  ================================================================
fprintf('\n=== 阶段4: 全局汇总 ===\n');

theta_vec = theta_opt;
v_vec     = v_opt;
T_by_missile = zeros(1, n_missiles);

for m = 1:n_missiles
    combined_tr = []; combined_dt = []; combined_uav = [];
    for k = 1:n_uavs
        n_b = length(tr_opt{k});
        if n_b > 0
            combined_tr  = [combined_tr,  tr_opt{k}];
            combined_dt  = [combined_dt,  dt_opt{k}];
            combined_uav = [combined_uav, repmat(k, 1, n_b)];
        end
    end
    if ~isempty(combined_tr)
        T_by_missile(m) = shielding_time(m, combined_uav, ...
            theta_vec, v_vec, combined_tr, combined_dt, params);
    end
end

fprintf('\n=== 各导弹最终遮蔽时间 ===\n');
fprintf('  M1: %.4f s\n', T_by_missile(1));
fprintf('  M2: %.4f s\n', T_by_missile(2));
fprintf('  M3: %.4f s\n', T_by_missile(3));
fprintf('  总计: %.4f s | 最小: %.4f s\n', sum(T_by_missile), min(T_by_missile));

%% 构建详细结果
fprintf('\n=== 各UAV最终策略 ===\n');
result = struct();
result.uav_results = cell(n_uavs, 1);
result.T_by_missile = T_by_missile;

for k = 1:n_uavs
    res_k = struct();
    res_k.uav_id = k;
    res_k.assigned_missile = assigned(k);
    res_k.theta  = theta_opt(k);
    res_k.v      = v_opt(k);
    res_k.t_releases = tr_opt{k};
    res_k.delta_ts   = dt_opt{k};
    res_k.t_dets     = tr_opt{k} + dt_opt{k};

    n_b = length(tr_opt{k});
    res_k.release_positions = zeros(n_b, 3);
    res_k.det_positions     = zeros(n_b, 3);

    uav_init = params.uavs(k, :);
    th_rad = deg2rad(theta_opt(k));
    uav_vel = [v_opt(k)*cos(th_rad), v_opt(k)*sin(th_rad), 0];

    for b = 1:n_b
        res_k.release_positions(b, :) = uav_traj(uav_init, ...
            theta_opt(k), v_opt(k), tr_opt{k}(b));
        res_k.det_positions(b, :) = bomb_traj(res_k.release_positions(b, :), ...
            uav_vel, tr_opt{k}(b), tr_opt{k}(b) + dt_opt{k}(b));
    end

    result.uav_results{k} = res_k;

    fprintf('FY%d→M%d: θ=%.0f° v=%.0fm/s %d弹 | ', ...
        k, assigned(k), theta_opt(k), v_opt(k), n_b);
    for b = 1:n_b
        fprintf('[弹%d: td=%.1fs 起爆(%.0f,%.0f,%.0f)] ', ...
            b, res_k.t_dets(b), res_k.det_positions(b, :));
    end
    fprintf('\n');
end

result.T_shield     = sum(T_by_missile);
result.T_shield_min = min(T_by_missile);
result.assigned_missile = assigned;
result.theta_opt = theta_opt;
result.v_opt     = v_opt;
result.tr_opt    = tr_opt;
result.dt_opt    = dt_opt;
end
