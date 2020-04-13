% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% ����˵�������������ڿ��������ԣ�ָ�������ɣ�SISO
function [sys,x0,str,ts] = TestExponentialApproachLawSMC(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % ���ó�ʼ���Ӻ���
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %��������Ӻ���
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
sizes.NumOutputs     = 2;  %�����������
sizes.NumInputs      = 2;   %�����������
sizes.DirFeedthrough = 1;   %�����ź��Ƿ�������Ӻ����г���
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %��ʼֵ
str = [];   
ts  = [];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %��������Ӻ���
%%����
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
