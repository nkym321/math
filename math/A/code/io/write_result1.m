function write_result1(result, filepath)
% write_result1 — 将问题3的结果写入 result1.xlsx
% 模板列：无人机运动方向 | 无人机运动速度(m/s) | 烟幕干扰弹编号 |
%          投放点x | 投放点y | 投放点z | 起爆点x | 起爆点y | 起爆点z |
%          有效干扰时间(s)
%
% 输入:
%   result   : solve_Q3 返回的结果结构体
%   filepath : 输出文件路径

fprintf('\n写入结果到 %s ...\n', filepath);

% 读取模板
template_path = strrep(filepath, 'output', '附件');
if ~exist(template_path, 'file')
    template_path = fullfile('..', '附件', 'result1.xlsx');
end

% 构建数据表
n_bombs = length(result.t_releases);

% 表头
headers = {'无人机运动方向', '无人机运动速度(m/s)', '烟幕干扰弹编号', ...
    '投放点x坐标(m)', '投放点y坐标(m)', '投放点z坐标(m)', ...
    '起爆点x坐标(m)', '起爆点y坐标(m)', '起爆点z坐标(m)', ...
    '有效干扰时间(s)'};

data = cell(n_bombs, 10);
for b = 1:n_bombs
    data{b, 1} = result.theta;        % 航向角(度)
    data{b, 2} = result.v;            % 速度(m/s)
    data{b, 3} = b;                   % 弹编号
    data{b, 4} = result.release_positions(b, 1);  % 投放点x
    data{b, 5} = result.release_positions(b, 2);  % 投放点y
    data{b, 6} = result.release_positions(b, 3);  % 投放点z
    data{b, 7} = result.det_positions(b, 1);      % 起爆点x
    data{b, 8} = result.det_positions(b, 2);      % 起爆点y
    data{b, 9} = result.det_positions(b, 3);      % 起爆点z
    data{b, 10} = result.T_shield;     % 有效遮蔽时间
end

% 写成表格并输出
T = cell2table(data, 'VariableNames', headers);
writetable(T, filepath, 'Sheet', 1);

fprintf('结果已写入 %s (%d行数据)\n', filepath, n_bombs);
end
