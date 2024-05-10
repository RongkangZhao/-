tic
% 初始化参数
T = 24; % 调度时段数量，一天24小时
LoadCurve = [0.594808046	0.564077723	0.537985547	0.534973266	0.527088521	0.538902477	0.588050794	0.677304591	0.760462995	0.800164898	0.818892957	0.822082205	0.714568123	0.739493004	0.784704406	0.805786312	0.850448839	0.913503039	0.937623975	0.900114146	0.857609882	0.799335702	0.737880619	0.659058
]*100; % 随机生成的负荷曲线
PriceCurve = [0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.3 0.3 0.3 0.4 0.4 0.4 0.4 0.4 0.3 0.3 0.3 0.5 0.5 0.5 0.3 0.3]; % 随机生成的电价曲线
Pmax = [10, 15]; % 可控电源最大出力，分别对应燃料电池和燃气轮机
C = [0.2, 0.15]; % 可控电源单位出力成本
SOCmin = 20; % 蓄电池最小SOC，单位为%
SOCmax = 100; % 蓄电池最大SOC，单位为%
ESS_capacity = 100; % 蓄电池容量，单位为kWh
ChargeRate = 10; % 蓄电池最大充电速率，单位为kW
DischargeRate = 10; % 蓄电池最大放电速率，单位为kW
% 边界条件
t=sdpvar(1,1);
soc_f=sdpvar(1,1);
L=sdpvar(1,1);
P=sdpvar(1,1);
% 决策变量
p1=sdpvar(1,1);
p2=sdpvar(1,1);
netPower=sdpvar(1,1);
essAction=sdpvar(1,1);
soc=sdpvar(1,1);
% 约束条件
Constraints = [];
Constraints = [Constraints;0<=p1<=10];
Constraints = [Constraints;0<=p2<=15];
Constraints = [Constraints;-1*DischargeRate<=essAction<=ChargeRate];
Constraints = [Constraints;p1+p2-essAction+netPower==L];
Constraints = [Constraints;soc-soc_f==essAction];
Constraints = [Constraints;20<=soc<=100];
% 目标函数及求解
Obj=0.2*p1+0.15*p2+P*netPower;
options=sdpsettings('solver','gurobi');
options=sdpsettings(options,'verbose',0);
out={Obj,p1,p2,netPower,essAction,soc};
M=optimizer(Constraints,Obj,options,{t,soc_f,L,P},out);
% 短视求解
for t=1:T
    L=LoadCurve(t);
    P=PriceCurve(t);
    if t==1
soc_f=50;
    end
    [Solutions,Diagnostics] = M({t,soc_f,L,P});
    cost_M(t)=Solutions{1};
    soc_M(t)=Solutions{6};
    soc_f=soc_M(t);
end
Cost_M=sum(cost_M);
toc