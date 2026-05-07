function pts = target_points(params)
% target_points — 生成真目标圆柱表面的离散采样点
% 输入:
%   params : 参数字典，包含 R_target, H_target, target_center, N_target_pts
% 输出:
%   pts    : N×3 矩阵，每行为一个目标表面采样点的 [x, y, z] 坐标 (m)
%
% 采样策略：在圆柱侧面、顶面和底面均匀采样，确保全面覆盖视线检测

R = params.R_target;
H = params.H_target;
cx = params.target_center(1);
cy = params.target_center(2);
cz = params.target_center(3);  % 底面z坐标

N = params.N_target_pts;
pts = zeros(N, 3);

% 圆柱参数方程：
% 侧面: x = cx + R*cos(phi), y = cy + R*sin(phi), z in [cz, cz+H]
% 顶面: x = cx + r*cos(phi), y = cy + r*sin(phi), z = cz+H, r in [0,R]
% 底面: x = cx + r*cos(phi), y = cy + r*sin(phi), z = cz, r in [0,R]

n_side = round(N * 0.5);   % 侧面采样点占50%
n_top  = round(N * 0.25);  % 顶面采样点占25%
n_bot  = N - n_side - n_top; % 底面采样点占25%

idx = 1;

% --- 侧面采样：多层圆环 ---
n_rings = ceil(sqrt(n_side));  % 高度方向层数
n_phi = ceil(n_side / n_rings); % 每层角度采样数
for ring = 1:n_rings
    z_val = cz + (ring - 0.5) / n_rings * H;  % 每层中心高度
    for i = 1:n_phi
        phi = (i - 1) / n_phi * 2 * pi;
        x_val = cx + R * cos(phi);
        y_val = cy + R * sin(phi);
        pts(idx, :) = [x_val, y_val, z_val];
        idx = idx + 1;
        if idx > n_side + 1
            break;
        end
    end
    if idx > n_side + 1
        break;
    end
end

% --- 顶面采样：同心圆环 ---
n_r_top = ceil(sqrt(n_top));
n_phi_top = ceil(n_top / n_r_top);
for r_idx = 1:n_r_top
    r_val = R * r_idx / n_r_top;
    for i = 1:n_phi_top
        phi = (i - 1) / n_phi_top * 2 * pi;
        x_val = cx + r_val * cos(phi);
        y_val = cy + r_val * sin(phi);
        pts(idx, :) = [x_val, y_val, cz + H];
        idx = idx + 1;
        if idx > n_side + n_top + 1
            break;
        end
    end
    if idx > n_side + n_top + 1
        break;
    end
end

% --- 底面采样：同心圆环 ---
n_r_bot = ceil(sqrt(n_bot));
n_phi_bot = ceil(n_bot / n_r_bot);
for r_idx = 1:n_r_bot
    r_val = R * r_idx / n_r_bot;
    for i = 1:n_phi_bot
        phi = (i - 1) / n_phi_bot * 2 * pi;
        x_val = cx + r_val * cos(phi);
        y_val = cy + r_val * sin(phi);
        pts(idx, :) = [x_val, y_val, cz];
        idx = idx + 1;
        if idx > N
            break;
        end
    end
    if idx > N
        break;
    end
end

% 截断到精确的N（处理舍入误差）
pts = pts(1:N, :);
end
