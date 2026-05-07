function write_result3(result, filepath)
% write_result3 — 将问题5的结果写入 result3.xlsx
% 模板列：无人机编号 | 无人机运动方向 | 无人机运动速度(m/s) |
%          烟幕干扰弹编号 | 投放点x | 投放点y | 投放点z |
%          起爆点x | 起爆点y | 起爆点z | 有效干扰时间(s) | 总的干扰成功率
%
% 输入:
%   result   : solve_Q5 返回的结果结构体
%   filepath : 输出文件路径

fprintf('\n写入结果到 %s ...\n', filepath);

headers = {'无人机编号', '无人机运动方向', '无人机运动速度(m/s)', ...
    '烟幕干扰弹编号', ...
    '投放点x坐标(m)', '投放点y坐标(m)', '投放点z坐标(m)', ...
    '起爆点x坐标(m)', '起爆点y坐标(m)', '起爆点z坐标(m)', ...
    '有效干扰时间(s)', '总的干扰成功率'};

% 汇总所有行
all_data = {};
for k = 1:length(result.uav_results)
    res_k = result.uav_results{k};
    n_bombs = length(res_k.t_releases);

    for b = 1:n_bombs
        row = cell(1, 12);
        row{1} = sprintf('FY%d', res_k.uav_id);
        row{2} = res_k.theta;
        row{3} = res_k.v;
        row{4} = b;
        row{5} = res_k.release_positions(b, 1);
        row{6} = res_k.release_positions(b, 2);
        row{7} = res_k.release_positions(b, 3);
        row{8} = res_k.det_positions(b, 1);
        row{9} = res_k.det_positions(b, 2);
        row{10} = res_k.det_positions(b, 3);
        row{11} = result.T_shield;
        % 成功率：简化计算为每枚弹的有效时间占比
        row{12} = min(1.0, result.T_shield / (20 * n_bombs));
        all_data(end+1, :) = row;
    end
end

T = cell2table(all_data, 'VariableNames', headers);
writetable(T, filepath, 'Sheet', 1);

fprintf('结果已写入 %s (%d行数据)\n', filepath, size(all_data, 1));
end
