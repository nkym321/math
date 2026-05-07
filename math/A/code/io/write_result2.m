function write_result2(result, filepath)
% write_result2 — 将问题4的结果写入 result2.xlsx
% 模板列：无人机编号 | 无人机运动方向 | 无人机运动速度(m/s) |
%          投放点x | 投放点y | 投放点z | 起爆点x | 起爆点y | 起爆点z |
%          有效干扰时间(s)
%
% 输入:
%   result   : solve_Q4 返回的结果结构体
%   filepath : 输出文件路径

fprintf('\n写入结果到 %s ...\n', filepath);

n_uavs = length(result.uav_ids);

headers = {'无人机编号', '无人机运动方向', '无人机运动速度(m/s)', ...
    '投放点x坐标(m)', '投放点y坐标(m)', '投放点z坐标(m)', ...
    '起爆点x坐标(m)', '起爆点y坐标(m)', '起爆点z坐标(m)', ...
    '有效干扰时间(s)'};

data = cell(n_uavs, 10);
for k = 1:n_uavs
    data{k, 1} = sprintf('FY%d', result.uav_ids(k));
    data{k, 2} = result.theta(k);
    data{k, 3} = result.v(k);
    data{k, 4} = result.release_positions(k, 1);
    data{k, 5} = result.release_positions(k, 2);
    data{k, 6} = result.release_positions(k, 3);
    data{k, 7} = result.det_positions(k, 1);
    data{k, 8} = result.det_positions(k, 2);
    data{k, 9} = result.det_positions(k, 3);
    data{k, 10} = result.T_shield;
end

T = cell2table(data, 'VariableNames', headers);
writetable(T, filepath, 'Sheet', 1);

fprintf('结果已写入 %s (%d行数据)\n', filepath, n_uavs);
end
