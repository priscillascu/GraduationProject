% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% 程序说明，本程序为使用改进趋近率的鲁棒滑模控制器
% 针对干扰上界已知的情况可以有效抑制干扰
function [sys,x0,str,ts] = ImprovedApproachLawSMC(t, x, u, flag)
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
sizes.NumInputs      = 12;   %输入变量个数
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
R1 = sin(0.5*t);;
R2 = sin(0.5*t);
R3 = sin(t);
R4 = sin(t);
R5 = sin(t);
R6 = sin(t);
R = [R1; R2; R3; R4; R5; R6];
%位置给定的导数，即速度
dR1 = 0.5*cos(0.5*t);
dR2 = 0.5*cos(0.5*t);
dR3 = cos(t);
dR4 = cos(t);
dR5 = cos(t);
dR6 = cos(t);
dR = [dR1; dR2; dR3; dR4; dR5; dR6];
%位置给定的二阶导数，即加速度
ddR1 = -0.25*sin(0.5*t);
ddR2 = -0.25*sin(0.5*t);
ddR3 = -sin(t);
ddR4 = -sin(t);
ddR5 = -sin(t);
ddR6 = -sin(t);
ddR = [ddR1; ddR2; ddR3; ddR4; ddR5; ddR6];
%%
%角度输入
th(1) = u(1);
th(2) = u(3);
th(3) = u(5);
th(4) = u(7);
th(5) = u(9);
th(6) = u(11);
%角速度输入
dth(1) = u(2);
dth(2) = u(4);
dth(3) = u(6);
dth(4) = u(8);
dth(5) = u(10);
dth(6) = u(12);
%%
%角度误差
e1 = R1 - th(1);
e2 = R2 - th(2);
e3 = R3 - th(3);
e4 = R4 - th(4);
e5 = R5 - th(5);
e6 = R6 - th(6);
e = [e1; e2; e3; e4; e5; e6];
%角速度误差
de1 = dR1 - dth(1);
de2 = dR2 - dth(2);
de3 = dR3 - dth(3);
de4 = dR4 - dth(4);
de5 = dR5 - dth(5);
de6 = dR6 - dth(6);
de = [de1; de2; de3; de4; de5; de6];

%%
%----定义旋转矩阵A并取绕Z轴旋转分量----
%部分DH参数
alpha_i_1 = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
theta = [th(1), th(2), th(3)-pi/2, th(4), th(5), th(6)];

A0i = 1;  %循环叠加求第i个连杆相对与0坐标系的旋转矩阵
e3 = [0; 0; 1];  %取Z轴分量
for j = 1 : 6
    A0i = A0i * [cos(theta(j)) -sin(theta(j)) 0;
                        cos(alpha_i_1(j))*sin(theta(j)) cos(alpha_i_1(j))*cos(theta(j)) -sin(alpha_i_1(j));
                        sin(alpha_i_1(j))*sin(theta(j)) sin(alpha_i_1(j))*cos(theta(j)) cos(alpha_i_1(j))];
    A0{j} = A0i;
end

m = [6.25, 5, 1.25, 0.9817, 1.25, 0.125]; %六个连杆的质量
%细胞数组J为各个连杆的转动惯量
J{1} = [ 0.0534           0           0 ;
            0           0.0534        0;
            0           0           0.0651];
        
J{2} = [0.0208        0           -0.0313;
            0           0.4208       0;
            -0.0313           0           0.4333];
        
J{3} = [0.0271       0           0;
            0           0.0021       0.0039;
            0           0.0039           0.0217];
        
J{4} = [0.0057      0           0;
            0           0.0057      0;
            0           0           0.0012];
        
J{5} = [0.0076      0           0;
            0           0.0021      0;
            0           0           0.0076];
        
J{6} = 1e-3.*[0.00015     0           0;
            0           0.00015     0;
            0           0           4e-05];
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
q{2} = [0; 0.125; 0];
q{3} = [0.5; 0; 0];
q{4} = [0; 0.25; 0];
q{5} = [0; 0; 0.125];
q{6} = [0; 0.125; 0];

r{1} = [0; 0; 0.05];
r{2} = [0.25; 0; 0.025];
r{3} = [0; 0.125; -0.025];
r{4} = [0; 0; 0.0625];
r{5} = [0; 0.0625; 0];
r{6} = [0; 0; 0.025];

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
%滑模控制器改进趋近率为ds = -eps*sign(s) -k1*abs(s)^alpha*sign(s) -
%k2*abs(s)^beta*sign(s)
%                                                      
c = [30; 30; 70; 70; 70; 300];
s = c.*e + de;
eps = 0.5;
k1 = [10; 10; 1; 1; 1; 1];
k2 = [20; 20; 1; 5; 5; 5];
alpha = [0.7; 0.7; 0.7; 0.9; 0.8; 0.7];
beta = [1; 2; 3; 3; 3; 3];
sigma = 0.5;
%tol = M*(ddR + c.*de + eps*s + k1.*abs(s).^(alpha).*tanh(s/sigma) + k2.*abs(s).^(beta).*tanh(s/sigma) + dc) + N*dth';
tol = M*(ddR + c.*de + eps*(s.^2).*tanh(s/sigma) + k1.*abs(s).^(alpha).*tanh(s/sigma) + k2.*abs(s).^(beta).*tanh(s/sigma)) + N*dth';

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