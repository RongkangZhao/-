tic
% 初始化参数
T = 24; % 调度时段数量，一天24小时
LoadCurve = [0.594808046	0.564077723	0.537985547	0.534973266	0.527088521	0.538902477	0.588050794	0.677304591	0.760462995	0.800164898	0.818892957	0.822082205	0.714568123	0.739493004	0.784704406	0.805786312	0.850448839	0.913503039	0.937623975	0.900114146	0.857609882	0.799335702	0.737880619	0.659058] * 100; % 负荷曲线
PriceCurve = [0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.3 0.3 0.3 0.4 0.4 0.4 0.4 0.4 0.3 0.3 0.3 0.5 0.5 0.5 0.3 0.3]; % 电价曲线
Pmax = [10, 15]; % 可控电源最大出力，分别对应燃料电池和燃气轮机
C = [0.2, 0.15]; % 可控电源单位出力成本
SOCmin = 20; % 蓄电池最小SOC，单位为%
SOCmax = 100; % 蓄电池最大SOC，单位为%
ChargeRate = 10; % 蓄电池最大充电速率，单位为kW
DischargeRate = 10; % 蓄电池最大放电速率，单位为kW
initialSOC = 50; % 初始SOC，单位为%

% 初始化动态规划表
CostTable = inf(T, SOCmax - SOCmin + 1); % 成本表
CostTable(1, initialSOC - SOCmin + 1) = 0; % 初始成本为0
ActionTable = zeros(T, SOCmax - SOCmin + 1); % 记录最优操作

% 前向计算
for t = 1:T-1
    for soc = SOCmin:SOCmax
        soc_idx = soc - SOCmin + 1;
        if CostTable(t, soc_idx) < inf  % 仅在成本有意义的情况下计算
            for p1 = 0:Pmax(1)
                for p2 = 0:Pmax(2)
                    for action = -DischargeRate:ChargeRate
                        newSOC = soc + action;
                        if newSOC >= SOCmin && newSOC <= SOCmax
                            netPower = p1 + p2 - LoadCurve(t) - action;
                            cost = C(1)*p1 + C(2)*p2 - PriceCurve(t)*netPower + CostTable(t, soc_idx);
                            new_idx = newSOC - SOCmin + 1;
                            if cost < CostTable(t+1, new_idx)
                                CostTable(t+1, new_idx) = cost;
                                ActionTable(t+1, new_idx) = action;
                            end
                        end
                    end
                end
            end
        end
    end
end

% 回溯找最优解
optimalActions = zeros(1, T);
[optimalCost, idx] = min(CostTable(T, :));
optimalSOC = zeros(1, T);
optimalSOC(T) = idx + SOCmin - 1;

for t = T:-1:2
    optimalActions(t) = ActionTable(t, idx);
    idx = idx - optimalActions(t);
    optimalSOC(t-1) = idx + SOCmin - 1;
end

% 输出结果
disp('Optimal SOC Path:');
disp(optimalSOC);
disp('Total Minimum Cost:');
disp(optimalCost);

% 结果可视化
plot(optimalSOC)
toc