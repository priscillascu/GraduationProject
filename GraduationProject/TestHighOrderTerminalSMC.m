function [sys,x0,str,ts] = TestHighOrderTerminalSMC(t, x, u, flag, th0)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % ���ó�ʼ���Ӻ���
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u, th0);    %��������Ӻ���
  case 4,
    sys=[];   %������һ����ʱ���Ӻ���
  case 9,
    sys=[];    %��ֹ�����Ӻ���
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %��ʼ���Ӻ���

sizes = simsizes;

sizes.NumContStates  = 0;  %����״̬��������
sizes.NumDiscStates  = 0;  %��ɢ״̬��������
sizes.NumOutputs     = 3;  %�����������
sizes.NumInputs      = 2;   %�����������
sizes.DirFeedthrough = 1;   %�����ź��Ƿ�������Ӻ����г���
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %��ʼֵ
str = [];   
ts  = [];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

global num;
num = 1;
global ctime;
ctime = 0;
function sys=mdlOutputs(t,x,u, th0)   %��������Ӻ���
global ctime;
ctime = ctime;
thd = sin(t);
dthd = cos(t);
ddthd = -sin(t);

f = -25*u(2);
b = 133;
D = 3;
F = 0;
K = 10;
C = [73, 1];

th = u(1);
dth = u(2);
e = th - thd;
de = dth - dthd;

th0 = th0;
dth0 = 0;
ddth0 = 0;
e0 = th0 - sin(0);
de0 = dth0 - cos(0);
dde0 = ddth0 + sin(0);

T = 5;
if(t > T)
    p = 0;
    dp = 0;
    ddp = 0;
else
    A1 = 10/(T^3)*e0 + 6/(T^2)*de0 + 3/(2*T)*dde0;
    B1 = 15/(T^4)*e0 + 8/(T^3)*de0 + 3/(2*(T^2))*dde0;
    C1 = 6/(T^5)*e0 + 3/(T^4)*de0 + 1/(2*(T^3))*dde0;
    p = e0  + de0*t + 1/2*dde0*t^2 - A1*t^3 + B1*t^4 - C1*t^5;
    dp = de0 + dde0*t - 3*A1*t^2 + 4*B1*t^3 - 5*C1*t^4;
    ddp = dde0 - 6*A1*t + 12*B1*t^2 - 20*C1*t^3;
end

E = [e; de];
P = [p; dp];
s = C*E - C*P;
ut = -1/b*(f - ddthd - ddp + 1/(C(2))*C(1)*(de - dp)) - 1/b*tanh(s/0.1)*(F+D+K);

global num;
if (abs(e) < 1e-04) && (abs(de) < 5e-03) && (num == 1);
    clc
    ctime = t
    
    num = 0;
end

sys(1) = ut;
sys(2) = e;
sys(3) = ctime;

