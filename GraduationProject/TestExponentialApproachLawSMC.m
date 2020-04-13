% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% 程序说明，本程序用于控制器测试，指数趋近律，SISO
function [sys,x0,str,ts] = TestExponentialApproachLawSMC(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % 调用初始化子函数
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %计算输出子函数
  case 4,
    sys=[];   %计算下一仿真时刻子函数
  case 9,
    sys=[];    %终止仿真子函数
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %初始化子函数

sizes = simsizes;

sizes.NumContStates  = 0;  %连续状态变量个数
sizes.NumDiscStates  = 0;  %离散状态变量个数
sizes.NumOutputs     = 2;  %输出变量个数
sizes.NumInputs      = 2;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %计算输出子函数
%%给定
thd = sin(t);
dthd = cos(t);
ddthd = -sin(t);

th = u(1);
dth = u(2);

c = 15;
e = thd - th;
de = dthd - dth;
s = c*e + de;

fx = 25*dth;
b = 133; 
epc = 1;
k = 10;

dU = 3;
dL = -3;
d1 = (dU - dL)/2;
d2 = (dU + dL)/2;
dc = d2 - d1*sign(s);

ut = 1/b*(epc*sign(s) + k*s + c*de + ddthd + fx - dc);

sys(1) = ut;
sys(2) = e;
