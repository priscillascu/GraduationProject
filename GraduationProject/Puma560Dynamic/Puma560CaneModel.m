% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% Puma560��е�ۿ�����������ѧ��ģ

function [sys,x0,str,ts] = Puma560CaneModel(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % ���ó�ʼ���Ӻ���
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

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %��ʼ���Ӻ���

sizes = simsizes;

sizes.NumContStates  = 12;  %����״̬��������
sizes.NumDiscStates  = 0;  %��ɢ״̬��������
sizes.NumOutputs     = 12;  %�����������
sizes.NumInputs      = 7;   %�����������
sizes.DirFeedthrough = 0;   %�����ź��Ƿ�������˳���
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [1 1 1 1 1 1 1 1 1 1 1 1];   %��ʼֵ
str = [];   
ts  = [0 0];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

function sys = mdlDerivatives(t, x, u)    %����΢���Ӻ���
%�����Ƕ�ֵ
th(1) = x(1);
th(2) = x(3);
th(3) = x(5);
th(4) = x(7);
th(5) = x(9);
th(6) = x(11);
%�������ٶ�ֵ
dth(1) = x(2);
dth(2) = x(4);
dth(3) = x(6);
dth(4) = x(8);
dth(5) = x(10);
dth(6) = x(12);
%��������
tol = u(1 : 6);

%----������ת����A��ȡ��Z����ת����----
for i = 1 : 6  %����6�����ű���theta1-theta6
    theta(i)=th(i);
end 

A0i = 1;  %ѭ���������i�����������0����ϵ����ת����
e3 = [0; 0; 1];  %ȡZ�����
for j = 1 : 6
    A0i = A0i * [cos(theta(j)) -sin(theta(j)) 0;
                        sin(theta(j)) cos(theta(j)) 0;
                        0 0 1];
    A0{j} = A0i;
end
m = [0, 17.4, 4.8, 0.82, 0.34, 0.09]; %�������˵�����
%ϸ������JΪ�������˵�ת������
% J{1} = [1.9627345061226189,0.058957330846854988, -0.78412289244774252;
%             0.058957330846854988, 2.60480624047246, -0.0028790724336223111;
%             -0.78412289244774252, -0.0028790724336223111, 1.8795078158813859];
% J{2} = [1.5382657404520751, -0.0034182963475227539, 0.021180594516188045;
%             -0.0034182963475227539, 1.4917350637236482, 0.0046805157642839631;
%             0.021180594516188045, 0.0046805157642839631, 0.080249418321229983];
% J{3} = [0.52446557260586973, -0.056641816223792746, 0.30329912935673073;
%             -0.056641816223792746, 0.73236553459530184, 0.05799389874290966;
%             0.30329912935673073, 0.05799389874290966, 0.51463961316216178];
% J{4} = [-0.084654168245494366, -0.0028735559189478622, 0.045090866312718092;
%             -0.0028735559189478622, 0.35548325308117125, 0.01685245190482066;
%             0.045090866312718092, 0.01685245190482066, 0.391710484509198];
% J{5} = [0.0098966460782527956, 0.00003026098309786931, 0.000028072719545672086;
%             0.00003026098309786931, 0.010025927190006987, -0.00080082417869195285;
%             0.000028072719545672086, -0.00080082417869195285, 0.011728543379556848];
% J{6} = [0.00113, 0, 0;
%             0, 0.000592, 0;
%             0, 0, 0.000578];
J{1} = [ 0           0           0 ;
            0           0.35        0;
            0           0           0];
J{2} = [0.13        0           0;
            0           0.524       0;
            0           0           0.539];
J{3} = [0.066       0           0;
            0           0.086       0;
            0           0           0.0125];
J{4} = [0.0018      0           0;
            0           0.0013      0;
            0           0           0.0018];
J{5} = [0.0003      0           0;
            0           0.0004      0;
            0           0           0.0003];
J{6} = [0.00015     0           0;
            0           0.00015     0;
            0           0           4e-05];
%����W����
W{1} = [e3'*A0{1}; 
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
        
W{2} = [e3'*A0{1}; 
            e3'*A0{2};
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
  
W{3} = [e3'*A0{1}; 
            e3'*A0{2};
            e3'*A0{3};
            0 0 0;
            0 0 0;
            0 0 0];
        
W{4} = [e3'*A0{1}; 
            e3'*A0{2};
            e3'*A0{3};
            e3'*A0{4};
            0 0 0;
            0 0 0];
        
W{5} = [e3'*A0{1}; 
            e3'*A0{2};
            e3'*A0{3};
            e3'*A0{4};
            e3'*A0{5};
            0 0 0];
        
W{6} = [e3'*A0{1}; 
            e3'*A0{2};
            e3'*A0{3};
            e3'*A0{4};
            e3'*A0{5};
            e3'*A0{6}];
        
WT{1} = W{1}';
WT{2} = W{2}';
WT{3} = W{3}';
WT{4} = W{4}';
WT{5} = W{5}';
WT{6} = W{6}';

%����V������Ҫ�õ����˳���q����λ��r
q{1} = [0; 0; 0];
q{2} = [0.4318; 0; 0];
q{3} = [0.0203; 0; 0];
q{4} = [0.4318; 0; 0];
q{5} = [0; 0; 0];
q{6} = [0; 0; 0];
r{1} = [0           0           0]';
r{2} = [-0.3638     0.006       0.2275]';
r{3} = [-0.0203     -0.0141     0.07]';
r{4} = [0           0.019       0]';
r{5} = [0           0           0]';
r{6} = [0           0           0.032]';

%���˾���ǰ�����������תΪ��˾���
%A0q�Ĳ�˾���
tmp1 = A0{1}*q{2};
tmp2 = A0{2}*q{3};
tmp3 = A0{3}*q{4};
tmp4 = A0{4}*q{5};
tmp5 = A0{5}*q{6};
A01q2x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
A02q3x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
A03q4x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
A04q5x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
A05q6x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];

%A0r�Ĳ�˾���
tmp1 = A0{1}*r{1};
tmp2 = A0{2}*r{2};
tmp3 = A0{3}*r{3};
tmp4 = A0{4}*r{4};
tmp5 = A0{5}*r{5};
tmp6 = A0{6}*r{6};
A01r1x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
A02r2x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
A03r3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
A04r4x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
A05r5x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
A06r6x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];
              
%����ViT����
VT{1} = -[A01r1x , [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{2} = -[A01q2x , A02r2x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{3} = -[A01q2x , A02q3x, A03r3x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{4} = -[A01q2x , A02q3x, A03q4x, A04r4x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{5} = -[A01q2x , A02q3x, A03q4x, A04q5x, A05r5x, [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{6} = -[A01q2x , A02q3x, A03q4x, A04q5x, A05q6x, A06r6x];

%A0e3�Ĳ�˾���
tmp1 = A0{1}*e3;
tmp2 = A0{2}*e3;
tmp3 = A0{3}*e3;
tmp4 = A0{4}*e3;
tmp5 = A0{5}*e3;
tmp6 = A0{6}*e3;
A01e3x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
A02e3x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
A03e3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
A04e3x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
A05e3x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
A06e3x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];

%����W�ĵ�������dW
dW{1} = [dth*W{1}*A01e3x 
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
        
dW{2} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];
        
dW{3} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            0 0 0;
            0 0 0;
            0 0 0];
        
dW{4} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            dth*W{4}*A04e3x;
            0 0 0;
            0 0 0];
        
dW{5} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            dth*W{4}*A04e3x;
            dth*W{5}*A05e3x;
            0 0 0];
        
dW{6} = [dth*W{1}*A01e3x; 
            dth*W{2}*A02e3x;
            dth*W{3}*A03e3x;
            dth*W{4}*A04e3x;
            dth*W{5}*A05e3x;
            dth*W{6}*A06e3x];
        
dWT{1} = dW{1}';
dWT{2} = dW{2}';
dWT{3} = dW{3}';
dWT{4} = dW{4}';
dWT{5} = dW{5}';
dWT{6} = dW{6}';

%omega_i�Ĳ�˾���
tmp1 = [0, 0, dth(1)];
tmp2 = [0, 0, dth(2)];
tmp3 = [0, 0, dth(3)];
tmp4 = [0, 0, dth(4)];
tmp5 = [0, 0, dth(5)];
tmp6 = [0, 0, dth(6)];
omega1x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
omega2x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
omega3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
omega4x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
omega5x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
omega6x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];
              
%omega_ixA0i-1qi�Ĳ�˾���
tmp1 = omega1x*A0{1}*q{2};
tmp2 = omega2x*A0{2}*q{3};
tmp3 = omega3x*A0{3}*q{4};
tmp4 = omega4x*A0{4}*q{5};
tmp5 = omega5x*A0{5}*q{6};
omega1xA01q2x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
omega2xA02q3x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
omega3xA03q4x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
omega4xA04q5x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
omega5xA05q6x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];       
              
%omega_ixA0iri�Ĳ�˾���
tmp1 = omega1x*A0{1}*r{1};
tmp2 = omega2x*A0{2}*r{2};
tmp3 = omega3x*A0{3}*r{3};
tmp4 = omega4x*A0{4}*r{4};
tmp5 = omega5x*A0{5}*r{5};
tmp6 = omega6x*A0{6}*r{6};
omega1xA01r1x = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
omega2xA02r2x = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
omega3xA03r3x = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
omega4xA04r4x = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
omega5xA05r5x = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
omega6xA06r6x = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];     
              
%����dViT����
dVT{1} = -[omega1xA01r1x , [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{2} = -[omega1xA01q2x , omega2xA02r2x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{3} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03r3x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{4} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04r4x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{5} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04q5x, omega5xA05r5x, [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{6} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04q5x, omega5xA05q6x, omega6xA06r6x];

%WkTdth�Ĳ�˾���
tmp1 = WT{1}*dth';
tmp2 = WT{2}*dth';
tmp3 = WT{3}*dth';
tmp4 = WT{4}*dth';
tmp5 = WT{5}*dth';
tmp6 = WT{6}*dth';
WTdth{1} = [0, -tmp1(3), tmp1(2);
                  tmp1(3), 0, -tmp1(1);
                  -tmp1(2), tmp1(1), 0];
              
WTdth{2} = [0, -tmp2(3), tmp2(2);
                  tmp2(3), 0, -tmp2(1);
                  -tmp2(2), tmp2(1), 0];
              
WTdth{3} = [0, -tmp3(3), tmp3(2);
                  tmp3(3), 0, -tmp3(1);
                  -tmp3(2), tmp3(1), 0];
              
WTdth{4} = [0, -tmp4(3), tmp4(2);
                  tmp4(3), 0, -tmp4(1);
                  -tmp4(2), tmp4(1), 0];
              
WTdth{5} = [0, -tmp5(3), tmp5(2);
                  tmp5(3), 0, -tmp5(1);
                  -tmp5(2), tmp5(1), 0];
              
WTdth{6} = [0, -tmp6(3), tmp6(2);
                  tmp6(3), 0, -tmp6(1);
                  -tmp6(2), tmp6(1), 0];     
%����M
M = 0;
for i = 1 : 6
    M = M + m(i)*(cell2mat(W)*VT{i}'*VT{i}*cell2mat(WT')) + W{i}*J{i}*W{i}';
end

%����N
N = 0;
for i = 1 : 6
    N = N + m(i)*(cell2mat(W)*VT{i}'*(dVT{i}*cell2mat(WT')+VT{i}*cell2mat(dWT')))+W{i}*J{i}*dWT{i}+W{i}*WTdth{i}*J{i}*WT{i};
end

%����ѧ����Mddth + Ndth = tol������dtΪ����Ŷ�
ddth = M\(tol - N*dth');

sys(1) = dth(1);
sys(2) = ddth(1);
sys(3) = dth(2);
sys(4) = ddth(2);
sys(5) = dth(3);
sys(6) = ddth(3);
sys(7) = dth(4);
sys(8) = ddth(4);
sys(9) = dth(5);
sys(10) = ddth(5);
sys(11) = dth(6);
sys(12) = ddth(6);

function sys=mdlOutputs(t,x,u)   %��������Ӻ���
sys(1) = x(1);  %λ�����
sys(2) = x(2);  %�ٶ����
sys(3) = x(3);  %λ�����
sys(4) = x(4);  %�ٶ����
sys(5) = x(5);  %λ�����
sys(6) = x(6);  %�ٶ����
sys(7) = x(7);  %λ�����
sys(8) = x(8);  %�ٶ����
sys(9) = x(9);  %λ�����
sys(10) = x(10);  %�ٶ����
sys(11) = x(11);  %λ�����
sys(12) = x(12);  %�ٶ����

