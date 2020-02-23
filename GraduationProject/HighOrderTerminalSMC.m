function [sys,x0,str,ts] = HighOrderTerminalSMC(t, x, u, flag)
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
sizes.NumOutputs     = 12;  %输出变量个数
sizes.NumInputs      = 18;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %计算输出子函数
%%给定
%角度给定
R1 = sin(t);
R2 = sin(t);
R3 = sin(t);
R4 = sin(t);
R5 = sin(t);
R6 = sin(t);
R = [R1; R2; R3; R4; R5; R6];
%位置给定的导数，即速度
dR1 = cos(t);
dR2 = cos(t);
dR3 = cos(t);
dR4 = cos(t);
dR5 = cos(t);
dR6 = cos(t);
dR = [dR1; dR2; dR3; dR4; dR5; dR6];
%位置给定的二阶导数，即加速度
ddR1 = -sin(t);
ddR2 = -sin(t);
ddR3 = -sin(t);
ddR4 = -sin(t);
ddR5 = -sin(t);
ddR6 = -sin(t);
ddR = [ddR1; ddR2; ddR3; ddR4; ddR5; ddR6];
%%

%%输入信号
%输入：反馈回来的角度信息
th(1) = u(1);
th(2) = u(4);
th(3) = u(7);
th(4) = u(10);
th(5) = u(13);
th(6) = u(16);
%输入：反馈回来的速度信息
dth(1) = u(2);
dth(2) = u(5);
dth(3) = u(8);
dth(4) = u(11);
dth(5) = u(14);
dth(6) = u(17);
%输入：反馈回来的加速度信息
ddth(1) = u(3);
ddth(2) = u(6);
ddth(3) = u(9);
ddth(4) = u(12);
ddth(5) = u(15);
ddth(6) = u(18);
%%

%%误差
%误差，位置误差
e1 = th(1) - R1;
e2 = th(2) - R2;
e3 = th(3) - R3;
e4 = th(4) - R4;
e5 = th(5) - R5;
e6 = th(6) - R6;
e = [e1; e2; e3; e4; e5; e6];
%误差一阶导数，速度误差
de1 = dth(1) - dR1;
de2 = dth(2) - dR2;
de3 = dth(3) - dR3;
de4 = dth(4) - dR4;
de5 = dth(5) - dR5;
de6 = dth(6) - dR6;
de = [de1; de2; de3; de4; de5; de6];
%误差二阶导数，加速度误差
dde1 = ddth(1) - ddR1;
dde2 = ddth(2) - ddR2;
dde3 = ddth(3) - ddR3;
dde4 = ddth(4) - ddR4;
dde5 = ddth(5) - ddR5;
dde6 = ddth(6) - ddR6;
dde = [dde1; dde2; dde3; dde4; dde5; dde6];

%%凯恩方法建模
%----定义旋转矩阵A并取绕Z轴旋转分量----
for i = 1 : 6  %创建6个符号变量theta1-theta6
    theta(i)=th(i);
end 

A0i = 1;  %循环叠加求第i个连杆相对与0坐标系的旋转矩阵
e3 = [0; 0; 1];  %取Z轴分量
for j = 1 : 6
    A0i = A0i * [cos(theta(j)) -sin(theta(j)) 0;
                        sin(theta(j)) cos(theta(j)) 0;
                        0 0 1];
    A0{j} = A0i;
end
m = [91.46742599797274 25.854095526361824 34.531766299390561 16.865972767053126 3.8797454961922262 0.6];  %六个连杆的质量
%细胞数组J为各个连杆的转动惯量
J{1} = [1.9627345061226189,0.058957330846854988, -0.78412289244774252;
            0.058957330846854988, 2.60480624047246, -0.0028790724336223111;
            -0.78412289244774252, -0.0028790724336223111, 1.8795078158813859];
J{2} = [1.5382657404520751, -0.0034182963475227539, 0.021180594516188045;
            -0.0034182963475227539, 1.4917350637236482, 0.0046805157642839631;
            0.021180594516188045, 0.0046805157642839631, 0.080249418321229983];
J{3} = [0.52446557260586973, -0.056641816223792746, 0.30329912935673073;
            -0.056641816223792746, 0.73236553459530184, 0.05799389874290966;
            0.30329912935673073, 0.05799389874290966, 0.51463961316216178];
J{4} = [-0.084654168245494366, -0.0028735559189478622, 0.045090866312718092;
            -0.0028735559189478622, 0.35548325308117125, 0.01685245190482066;
            0.045090866312718092, 0.01685245190482066, 0.391710484509198];
J{5} = [0.0098966460782527956, 0.00003026098309786931, 0.000028072719545672086;
            0.00003026098309786931, 0.010025927190006987, -0.00080082417869195285;
            0.000028072719545672086, -0.00080082417869195285, 0.011728543379556848];
J{6} = [0.00113, 0, 0;
            0, 0.000592, 0;
            0, 0, 0.000578];

%创建W矩阵
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

%创建V矩阵，需要用到连杆长度q质心位置r
q{1} = [0; 0; 0];
q{2} = [0.1949; 0.0169; 0.2850];
q{3} = [0.6137; 1.0938e-04; 0.0030];
q{4} = [0.2000; 0.2750; 0.1105];
q{5} = [7.6891e-04; -0.0655; 0.3663];
q{6} = [4.7162e-04; 0.0981; -0.0540];
r{1} = [0.0660, -0.0052, 0.1156]';
r{2} = [0.2894, -0.0155, -0.0268]';
r{3} = [0.1212, 0.1196, 0.1147]';
r{4} = [0.0294, 0.0065, 0.1947]';
r{5} = [-4.7034e-05, 0.0497, -0.0576]';
r{6} = [-0.0039, -0.0019, -0.0539]';

%求叉乘矩阵前，先求积，再转为叉乘矩阵
%A0q的叉乘矩阵
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

%A0r的叉乘矩阵
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
              
%创建ViT矩阵
VT{1} = -[A01r1x , [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{2} = -[A01q2x , A02r2x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{3} = -[A01q2x , A02q3x, A03r3x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{4} = -[A01q2x , A02q3x, A03q4x, A04r4x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{5} = -[A01q2x , A02q3x, A03q4x, A04q5x, A05r5x, [0, 0, 0; 0, 0, 0; 0, 0, 0]];
VT{6} = -[A01q2x , A02q3x, A03q4x, A04q5x, A05q6x, A06r6x];

%A0e3的叉乘矩阵
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

%创建W的导数矩阵dW
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

%omega_i的叉乘矩阵
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
              
%omega_ixA0i-1qi的叉乘矩阵
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
              
%omega_ixA0iri的叉乘矩阵
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
              
%创建dViT矩阵
dVT{1} = -[omega1xA01r1x , [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{2} = -[omega1xA01q2x , omega2xA02r2x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{3} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03r3x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{4} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04r4x, [0, 0, 0; 0, 0, 0; 0, 0, 0], [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{5} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04q5x, omega5xA05r5x, [0, 0, 0; 0, 0, 0; 0, 0, 0]];
dVT{6} = -[omega1xA01q2x , omega2xA02q3x, omega3xA03q4x, omega4xA04q5x, omega5xA05q6x, omega6xA06r6x];

%WkTdth的叉乘矩阵
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
%参数M
M = 0;
for i = 1 : 6
    M = M + m(i)*(cell2mat(W)*VT{i}'*VT{i}*cell2mat(WT')) + W{i}*J{i}*W{i}';
end

%参数N
N = 0;
for i = 1 : 6
    N = N + m(i)*(cell2mat(W)*VT{i}'*(dVT{i}*cell2mat(WT')+VT{i}*cell2mat(dWT')))+W{i}*J{i}*dWT{i}+W{i}*WTdth{i}*J{i}*WT{i};
end

%%
%Termianl滑模控制器
%0时刻的误差值e0
% if t == 0
%     e0 = th';  %0时刻的误差为th - sin(0) = th
%     de0 = dth' - [1; 1; 1; 1; 1; 1];  %0时刻的误差一阶导数为dth - cos(0) = dth - 1
%     dde0 = ddth';  %0时刻的误差二阶导数为ddth + sin(0) = th
% end
e0 = [0; 0; 0; 0; 0; 0];
de0 = [-1; -1; -1; -1; -1; -1];
dde0 = [0; 0; 0; 0; 0; 0];
%P算子中的系数a
T = 0.1;
a00 = -10;
a10 = 15;
a20 = -6;

a01 = -6;
a11 = 8;
a21 = -3;

a02 = -3/2;
a12 = 3/2;
a22 = -1/2;

if(t > T)
    p = [0; 0; 0; 0; 0; 0];
    dp = [0; 0; 0; 0; 0; 0];
    ddp = [0; 0; 0; 0; 0; 0];
else   
    A1 = a00/(T^3)*e0 + a01/(T^2)*de0 + a02/T*dde0;  %t^3的系数项
    B1 = a10/(T^4)*e0 + a11/(T^3)*de0 + a12/(T^2)*dde0;  %t^4的系数项
    C1 = a20/(T^5)*e0 + a21/(T^4)*de0 + a22/(T^3)*dde0;  %t^5的系数项
    p = e0 + de0*t + 1/2*dde0*t^2 + A1*t^3 + B1*t^4 + C1*t^5;
    dp = de0 + dde0*t + 3*A1*t^2 + 4*B1*t^3 + 5*C1*t^4;
    ddp = dde0 + 6*A1*t + 12*B1*t^2 + 20*C1*t^3;
end

C1 = diag([4, 4, 4, 4, 4, 4], 0);
C2 = diag([4, 4, 4, 4, 4, 4], 0);
C = [C1, C2];
E = [e; de];
P = [p; dp];
s = C*E - C*P;
b = inv(M);
f = -N/M*dth';
F = 5;
D = 5;
K = 10;
ita =  D + 0.1;
% tol = -M*(f - ddR - ddp + inv(C2)*C1*(de - dp)) - M*sign(C2*s)*(F+D+K);
tol = -M*(inv(C1)*(de - dp) + f - ddR - ddp + ita*sign(s)*(F + D + K));

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