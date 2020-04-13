% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% ���ڿ����㷨���Ե�ģ�ͣ�SISO
function [sys,x0,str,ts] = ControlTestPlant(t, x, u, flag, x00)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(x00);  % ���ó�ʼ���Ӻ���
  case 1,
    sys=mdlDerivatives(t,x,u);   %���ü���΢���Ӻ���
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

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(x00)   %��ʼ���Ӻ���

sizes = simsizes;

sizes.NumContStates  = 2;  %����״̬��������
sizes.NumDiscStates  = 0;  %��ɢ״̬��������
sizes.NumOutputs     = 2;  %�����������
sizes.NumInputs      = 1;   %�����������
sizes.DirFeedthrough = 0;   %�����ź��Ƿ�������˳���
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [x00 0];   %��ʼֵ
str = [];   
ts  = [0 0];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

function sys = mdlDerivatives(t, x, u)    %����΢���Ӻ���
dt = 3*sin(2*pi*t);

sys(1) = x(2);
sys(2) = -25*x(2) + 133*u + dt;

function sys=mdlOutputs(t,x,u)   %��������Ӻ���
sys(1) = x(1);  
sys(2) = x(2);  

