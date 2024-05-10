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
H=4; % 视窗
soc_f=sdpvar(1,1);
L=sdpvar(1,H);
P=sdpvar(1,H);
% 决策变量
p1=sdpvar(1,H);
p2=sdpvar(1,H);
netPower=sdpvar(1,H);
essAction=sdpvar(1,H);
soc=sdpvar(1,H);
% 约束条件
Constraints = [];
Constraints = [Constraints;0<=p1<=repmat(10,1,H)];
Constraints = [Constraints;0<=p2<=repmat(15,1,H)];
Constraints = [Constraints;-1.*repmat(DischargeRate,1,H)<=essAction<=repmat(ChargeRate,1,H)];
Constraints = [Constraints;p1+p2-essAction+netPower==L];
Constraints = [Constraints;soc-[soc_f soc(1:end-1)]==essAction];
Constraints = [Constraints;repmat(20,1,H)<=soc<=repmat(100,1,H)];
% 目标函数及求解
Obj=sum(0.2.*p1)+sum(0.15.*p2)+sum(P.*netPower);
fval=0.2.*p1+0.15.*p2+P.*netPower;
options=sdpsettings('solver','gurobi');
options=sdpsettings(options,'verbose',0);
out={Obj,p1,p2,netPower,essAction,soc,fval};
MPC=optimizer(Constraints,Obj,options,{soc_f,L,P},out);
% MPC求解
for t=1:T-H+1
    L=LoadCurve(t:t+H-1);
    P=PriceCurve(t:t+H-1);
    if t==1
soc_f=50;
    end
    [Solutions,Diagnostics] = MPC({soc_f,L,P});
    cost=Solutions{7};
    soc=Solutions{6};
    cost_MPC(t)=cost(:,1);
    soc_MPC(t)=soc(:,1);
    soc_f=soc_MPC(t);
end
% 剩余时刻赋值
for j=2:H
    cost_MPC(t+j-1)=cost(:,j);
    soc_MPC(t+j-1)=soc(:,j);
end
Cost_MPC=sum(cost_MPC);
toc