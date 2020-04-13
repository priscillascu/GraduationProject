function [sys,x0,str,ts] = TestInfiniteTimeSMC(t, x, u, flag)
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
sizes.NumOutputs     = 1;  %输出变量个数
sizes.NumInputs      = 2;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

global num;
num = 1;
global ctime;
ctime = 0;
function sys=mdlOutputs(t,x,u)   %计算输出子函数
th = u(1);
dth = u(2);
thd = sin(t);
dthd = cos(t);
ddthd = -sin(t);

e = th - thd;
de = dth - dthd;

x1 = th - thd;
x2 = dth - dthd;
mc = 1;
m = 0.1;
l = 0.5;
g = 9.8;
D = 2;

% T = l*(4/3-m*cos(x1)*cos(x1)/(mc+m));
% fx = g*sin(x1)-m*l*x2*x2*cos(x1)*sin(x1)/(mc+m);
% fx = fx/T;
% gx = cos(x1)/(mc+m);
% gx = gx/T;
fx = -25*(de + dthd) - ddthd;
gx = 133;

q = 3;
p = 5;
beta = 1;
xite = 2;

flag = 1;

if flag == 0    %奇异问题，当x1 = 0的时候会报错
    T1 = abs(e)^(q/p)*sign(e);
    T2 = abs(e)^(q/p-1)*sign(e);   %若直接用x2^(2-p/q)则会出现计算错误，因为2-p/q可能为开根，x2可能小于0
    s = de + beta*T1;
    ut = -(1/gx)*(fx + (beta*q/p)*T2*de + (D+xite)*sign(s));
else
    T1 = abs(de)^(p/q)*sign(de);
    T2 = abs(de)^(2-p/q)*sign(de);   %若直接用x2^(2-p/q)则会出现计算错误，因为2-p/q可能为开根，x2可能小于0
    s = e + 1/beta*T1;
    ut = -(1/gx)*(fx + (beta*q/p)*T2 + (D+xite)*sign(s));
end

global num;
if (abs(e) < 1e-04) && (abs(de) < 5e-03) && (num == 1);
    clc
    ctime = t
    
    num = 0;
end

sys(1) = ut;



