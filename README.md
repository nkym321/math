# 2025 年全国大学生数学建模竞赛 A题 — 烟幕干扰弹的投放策略

2025 年高教社杯全国大学生数学建模竞赛（CUMCM）**A题**的源代码与论文。

## 问题概述

敌方空地导弹向保护目标来袭，挂载烟幕干扰弹的无人机需要在导弹与目标之间投放干扰弹，形成烟幕遮蔽，最大化有效遮蔽时长。核心挑战：在运动学约束和投放间隔约束下，优化无人机的飞行方向、速度、干扰弹投放点和起爆时间。

- **3枚来袭导弹**（M1、M2、M3），以 300 m/s 的速度飞向位于原点的假目标
- **5架无人机**（FY1–FY5），等高度巡航，速度可调（70–140 m/s）
- 每架无人机最多挂载 3 枚干扰弹，连续投放间隔 ≥ 1 s
- 干扰弹投放后自由落体，起爆后瞬时形成球状烟幕云团（半径 10 m，以 3 m/s 匀速下沉，有效期 20 s）
- 真目标为圆柱体（半径 7 m，高 10 m），底面圆心位于 (0, 200, 0)

### 五个子问题

| 问题 | 描述 | 求解方法 |
|------|------|----------|
| **问题1** | 确定性计算：FY1 投放 1 枚弹对 M1，参数固定 | 运动学仿真 |
| **问题2** | 优化 FY1 投放 1 枚弹对 M1（4 个决策变量） | 网格搜索 + SQP |
| **问题3** | 优化 FY1 投放 3 枚弹对 M1 | 遗传算法 + SQP |
| **问题4** | 协调 FY1、FY2、FY3 各投放 1 枚弹对 M1 | 遗传算法 |
| **问题5** | 全场景：5 架无人机各最多 3 枚弹对 3 枚导弹（40 个变量） | GA + 分解预分配 |

## 目录结构

```
A/
├── code/                       # MATLAB 源代码
│   ├── main.m                  # 主控脚本，依次求解所有子问题
│   ├── config.m                # 物理常数与初始位置配置
│   ├── kinematics/             # 运动学模型
│   │   ├── missile_traj.m      #   导弹直线飞行轨迹
│   │   ├── uav_traj.m          #   无人机等高度飞行轨迹
│   │   ├── bomb_traj.m         #   干扰弹自由落体轨迹
│   │   └── smoke_cloud.m       #   烟幕云团下沉运动
│   ├── geometry/               # 几何遮挡模型
│   │   ├── target_points.m     #   圆柱目标离散化采样
│   │   ├── los_blocked.m       #   视线遮挡判断
│   │   └── shielding_time.m    #   有效遮蔽时间计算
│   ├── optimization/           # 优化求解器
│   │   ├── objective_Q3.m      #   问题3目标函数
│   │   ├── objective_Q4.m      #   问题4目标函数
│   │   ├── objective_Q5.m      #   问题5目标函数
│   │   ├── constraints.m       #   约束函数
│   │   └── ga_wrapper.m        #   遗传算法封装（含 SQP 精化）
│   ├── solve_Q1.m .. solve_Q5.m  # 各问题求解脚本
│   ├── io/                     # 结果输出
│   │   ├── write_result1.m     #   写入问题3结果至 Excel
│   │   ├── write_result2.m     #   写入问题4结果至 Excel
│   │   └── write_result3.m     #   写入问题5结果至 Excel
│   └── visualize/              # 可视化
│       ├── plot_trajectories.m #   三维轨迹总览图
│       ├── plot_shielding.m    #   遮蔽状态分析图
│       └── plot_convergence.m  #   优化收敛曲线图
├── paper/                      # LaTeX 论文
│   ├── main.tex                # 主文档
│   ├── refs.bib                # 参考文献
│   └── sections/               # 章节文件（共 9 节）
├── output/                     # 生成的结果文件
│   ├── result1.xlsx            # 问题3输出
│   ├── result2.xlsx            # 问题4输出
│   ├── result3.xlsx            # 问题5输出
│   ├── all_results.mat         # 完整 MATLAB 结果数据
│   └── figures/                # 生成的可视化图表
├── 附件/                       # 竞赛官方附件
└── A题.pdf                     # 竞赛原题 PDF
```

## 方法论

### 运动学建模
- **导弹：** 匀速直线飞行，方向指向假目标（原点）
- **无人机：** 受领任务后瞬时调整航向，之后以恒定速度等高度直线飞行
- **干扰弹：** 投放后在重力作用下自由落体（继承无人机水平速度分量）
- **烟幕云团：** 起爆后呈球状，以 3 m/s 匀速下沉，半径 10 m 内有效期 20 s

### 视线遮挡判断
将圆柱形真目标表面离散化为采样点集。对于每个采样点，计算导弹到该点的线段是否与烟幕球体相交——通过解析几何求解线段到球心的最近距离。当且仅当**所有**采样点均被至少一个有效烟幕云团遮挡时，判定目标处于完全遮蔽状态。

### 分层优化策略
根据问题复杂度采用递进式求解策略：
1. **问题1：** 确定性仿真计算
2. **问题2：** 网格搜索 + 序列二次规划（SQP）混合优化
3. **问题3–5：** 遗传算法（GA）全局搜索 + SQP 局部精化
4. **问题5：** 额外引入基于无人机-导弹距离的分解预分配策略，应对 40 维搜索空间

## 使用说明

### 环境要求
- **MATLAB**（建议 R2020a 及以上版本）
  - Global Optimization Toolbox（提供 `ga` 函数）
  - Optimization Toolbox（提供 `fmincon` / SQP）
- **LaTeX**（TeX Live 或 MiKTeX，需含 `ctexart` 文档类）— 用于编译论文

### 运行代码

```matlab
% 在 MATLAB 中切换至 code/ 目录，运行：
main
```

脚本将依次求解所有子问题，并将结果写入 `output/` 目录。

### 编译论文

```bash
cd paper
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

## 主要结果

结果按照官方模板格式写入 `output/result1.xlsx` 至 `result3.xlsx`。`output/figures/` 目录包含以下可视化图表：

- **trajectories_overview.png** — 无人机航迹、干扰弹轨迹与导弹轨迹的三维总览
- **shielding_analysis.png** — 各子问题的遮蔽状态时间线
- **convergence.png** — 遗传算法收敛曲线

## 许可证

本项目为 2025 年 CUMCM 参赛学术作品，可供数学建模学习参考使用。

## 致谢

- 2025 年全国大学生数学建模竞赛组委会提供赛题
- MATLAB 工具箱：Global Optimization Toolbox、Optimization Toolbox
