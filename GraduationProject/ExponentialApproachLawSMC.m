% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% ����˵����������Ϊʹ��ָ�������ʵ�³����ģ������
function [sys,x0,str,ts] = ExponentialApproachLawSMC(t, x, u, flag)
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
sizes.NumOutputs     = 12;  %�����������
sizes.NumInputs      = 12;   %�����������
sizes.DirFeedthrough = 1;   %�����ź��Ƿ�������Ӻ����г���
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %��ʼֵ
str = [];   
ts  = [];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %��������Ӻ���
%%����
%�Ƕȸ���
R1 = cos(0.5*t);
R2 = cos(0.5*t);
R3 = cos(t);
R4 = cos(t);
R5 = cos(t);
R6 = cos(t);
R = [R1; R2; R3; R4; R5; R6];
%λ�ø����ĵ��������ٶ�
dR1 = -0.5*sin(0.5*t);
dR2 = -0.5*sin(0.5*t);
dR3 = -sin(t);
dR4 = -sin(t);
dR5 = -sin(t);
dR6 = -sin(t);
dR = [dR1; dR2; dR3; dR4; dR5; dR6];
%λ�ø����Ķ��׵����������ٶ�
ddR1 = -0.25*cos(0.5*t);
ddR2 = -0.25*cos(0.5*t);
ddR3 = -cos(t);
ddR4 = -cos(t);
ddR5 = -cos(t);
ddR6 = -cos(t);
ddR = [ddR1; ddR2; ddR3; ddR4; ddR5; ddR6];
%%
%�Ƕ�����
th(1) = u(1);
th(2) = u(3);
th(3) = u(5);
th(4) = u(7);
th(5) = u(9);
th(6) = u(11);
%���ٶ�����
dth(1) = u(2);
dth(2) = u(4);
dth(3) = u(6);
dth(4) = u(8);
dth(5) = u(10);
dth(6) = u(12);
%%
%�Ƕ����
e1 = R1 - th(1);
e2 = R2 - th(2);
e3 = R3 - th(3);
e4 = R4 - th(4);
e5 = R5 - th(5);
e6 = R6 - th(6);
e = [e1; e2; e3; e4; e5; e6];
%���ٶ����
de1 = dR1 - dth(1);
de2 = dR2 - dth(2);
de3 = dR3 - dth(3);
de4 = dR4 - dth(4);
de5 = dR5 - dth(5);
de6 = dR6 - dth(6);
de = [de1; de2; de3; de4; de5; de6];

%%
%----������ת����A��ȡ��Z����ת����----
%����DH����
alpha_i_1 = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
theta = [th(1), th(2), th(3)-pi/2, th(4), th(5), th(6)];

A0i = 1;  %ѭ���������i�����������0����ϵ����ת����
e3 = [0; 0; 1];  %ȡZ�����
for j = 1 : 6
    A0i = A0i * [cos(theta(j)) -sin(theta(j)) 0;
                        cos(alpha_i_1(j))*sin(theta(j)) cos(alpha_i_1(j))*cos(theta(j)) -sin(alpha_i_1(j));
                        sin(alpha_i_1(j))*sin(theta(j)) sin(alpha_i_1(j))*cos(theta(j)) cos(alpha_i_1(j))];
    A0{j} = A0i;
end

m = [17.9513, 3.5484, 7.3201, 3.8682, 0.7287, 1]; %�������˵�����
%ϸ������JΪ�������˵�ת������
J{1} = [ 0.5354           0.0131           -0.2059;
            0.0131           0.7118        0.0404;
            -0.2059           0.0404           0.5010];
        
J{2} = [0.5044, -0.0164, -0.0021;
    -0.0164, 0.0144, -0.0304;
    -0.0021, -0.0304, 0.5091];
        
J{3} = [0.2601, -0.1844, -0.0883;
    -0.1844, 0.2780, -0.0850;
    -0.0883, -0.0850, 0.3979];
        
J{4} = [0.1544, -0.0001, -0.0143;
    -0.0001, 0.1527, -0.0051;
    -0.0143, -0.0051, 0.0224];
        
J{5} = [0.0055, 0, 0;
    0, 0.0040, -0.0015;
    0, -0.0015, 0.0028];
        
J{6} = [0.0042, 0, 0;
    0, 0.0042, 0;
    0, 0, 0.0017];
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
q{2} = [0.1949; -0.0951; 0.2850];
q{3} = [1.0938e-04; -0.6137; 0.0030];
q{4} = [0.2000; 0.2750; 0.1105];
q{5} = [0; 0.0320; 0.3650];
q{6} = [-4.7162e-04; 0.0981; 0.0540];

r{1} = [0.0467; -0.0104; 0.1100];
r{2} = [-0.0155; -0.2895; -0.0268];
r{3} = [0.1366; 0.1311; 0.0806];
r{4} = [0.0350; 0.0044; 0.1474];
r{5} = [3.9663e-06; -0.0019; 0.0540];
r{6} = [0; 0; 0.05];

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

%%
%ָ�������ʻ�ģ������ds = -eps*sat(s) - ks��s = alpha*e + de
%ͨ������k��alpha�����Ը���ϵͳ����Ӧ�ٶȺ͸���������ᱬը
%���󱥺ͺ���sat�ı߽��delta�����Լ�Сϵͳ�Ķ���
%eps���ڲ�����ȷ����E(t)�����ϴ��eps����ɶ��񣬺�̿ɲ���ģ�����ƶ�̬����
alpha = [68; 63; 65; 125; 208; 208];
eps = [8; 8; 8; 10; 10; 10];
k = [21; 30; 18; 18; 33; 28];
s = alpha.*e + de;
%���ñ��ͺ���sat������ź���sign
delta = 30;
kk = 1/delta;
if abs(s) > delta
    sats = sign(s);
else
    sats = kk*s;
end

tol = M*(alpha.*de + ddR + eps.*sats + k.*s) + N*dth';
% sds = -eps.*s.*sats - k.*(s.^2)  %�������ģ��ϵͳ����������

sys(1) = tol(1);
sys(2) = tol(2);
sys(3) = tol(3);
sys(4) = tol(4);
sys(5) = tol(5);
sys(6) = tol(6);
sys(7) = e(1);
sys(8) = e(2);
sys(9) = e(3);
sys(10) = e(4);
sys(11) = e(5);
sys(12) = e(6);