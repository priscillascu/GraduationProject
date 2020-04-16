clear;
clc;
%第一个关节的初始、终止角度
q01 = 0;
qf1 = 0.3131; 
%第二个关节的初始、终止角度
q02 = 0;
qf2 = -0.3181; 
%第三个关节的初始、终止角度
q03 = 0;
qf3 = 0.5993; 
%第四个关节的初始、终止角度
q04 = 0;
qf4 = 0.8623; 
%第五个关节的初始、终止角度
q05 = 0;
qf5 = -0.4178; 
%第六个关节的初始、终止角度
q06 = 0;
qf6 = -0.8176; 

%指定起止时间
t0 = 0;
tf = 10;
%指定起止速度、加速度
v0 = 0;
vf = 0;
a0 = 0;
af = 0;

%计算第1个关节的五次多项式系数
a10 = q01;
a11 = v0;
a12=a0/2;
a13 = (20*qf1 - 20*q01 - (8*vf + 12*v0)*tf - (3*a0 - af)*tf^2)/(2*tf^3);
a14 = (30*q01 - 30*qf1 + (14*vf + 16*v0)*tf + (3*a0 - 2*af)*tf^2)/(2*tf^4);
a15 = (12*qf1 - 12*q01 - (6*vf + 6*v0)*tf - (a0 - af)*tf^2)/(2*tf^5);
%计算第2个关节的五次多项式系数
a20 = q02;
a21 = v0;
a22=a0/2;
a23 = (20*qf2 - 20*q02 - (8*vf + 12*v0)*tf - (3*a0 - af)*tf^2)/(2*tf^3);
a24 = (30*q02 - 30*qf2 + (14*vf + 16*v0)*tf + (3*a0 - 2*af)*tf^2)/(2*tf^4);
a25 = (12*qf2 - 12*q02 - (6*vf + 6*v0)*tf - (a0 - af)*tf^2)/(2*tf^5);
%计算第3个关节的五次多项式系数
a30 = q03;
a31 = v0;
a32=a0/2;
a33 = (20*qf3 - 20*q03 - (8*vf + 12*v0)*tf - (3*a0 - af)*tf^2)/(2*tf^3);
a34 = (30*q03 - 30*qf3 + (14*vf + 16*v0)*tf + (3*a0 - 2*af)*tf^2)/(2*tf^4);
a35 = (12*qf3 - 12*q03 - (6*vf + 6*v0)*tf - (a0 - af)*tf^2)/(2*tf^5);
%计算第4个关节的五次多项式系数
a40 = q04;
a41 = v0;
a42=a0/2;
a43 = (20*qf4 - 20*q04 - (8*vf + 12*v0)*tf - (3*a0 - af)*tf^2)/(2*tf^3);
a44 = (30*q04 - 30*qf4 + (14*vf + 16*v0)*tf + (3*a0 - 2*af)*tf^2)/(2*tf^4);
a45 = (12*qf4 - 12*q04 - (6*vf + 6*v0)*tf - (a0 - af)*tf^2)/(2*tf^5);
%计算第5个关节的五次多项式系数
a50 = q05;
a51 = v0;
a52=a0/2;
a53 = (20*qf5 - 20*q05 - (8*vf + 12*v0)*tf - (3*a0 - af)*tf^2)/(2*tf^3);
a54 = (30*q05 - 30*qf5 + (14*vf + 16*v0)*tf + (3*a0 - 2*af)*tf^2)/(2*tf^4);
a55 = (12*qf5 - 12*q05 - (6*vf + 6*v0)*tf - (a0 - af)*tf^2)/(2*tf^5);
%计算第6个关节的五次多项式系数
a60 = q06;
a61 = v0;
a62=a0/2;
a63 = (20*qf6 - 20*q06 - (8*vf + 12*v0)*tf - (3*a0 - af)*tf^2)/(2*tf^3);
a64 = (30*q06 - 30*qf6 + (14*vf + 16*v0)*tf + (3*a0 - 2*af)*tf^2)/(2*tf^4);
a65 = (12*qf6 - 12*q06 - (6*vf + 6*v0)*tf - (a0 - af)*tf^2)/(2*tf^5);

t = t0:0.01:tf;
%第1个关节的角度、角速度、角加速度
q1 = a10+a11*t+a12*t.^2+a13*t.^3 + a14*t.^4 + a15*t.^5;
v1 = a11+2*a12*t+3*a13*t.^2 + 4*a14*t.^3 + 5*a15*t.^4;
a1 = 2*a12+6*a13*t + 12*a14*t.^2 + 20*a15*t.^3;
%第2个关节的角度、角速度、角加速度
q2 = a20+a21*t+a22*t.^2+a23*t.^3 + a24*t.^4 + a25*t.^5;
v2 = a21+2*a22*t+3*a23*t.^2 + 4*a24*t.^3 + 5*a25*t.^4;
a2 = 2*a22+6*a23*t + 12*a24*t.^2 + 20*a25*t.^3;
%第3个关节的角度、角速度、角加速度
q3 = a30+a31*t+a32*t.^2+a33*t.^3 + a34*t.^4 + a35*t.^5;
v3 = a31+2*a32*t+3*a33*t.^2 + 4*a34*t.^3 + 5*a35*t.^4;
a3 = 2*a32+6*a33*t + 12*a34*t.^2 + 20*a35*t.^3;
%第4个关节的角度、角速度、角加速度
q4 = a40+a41*t+a42*t.^2+a43*t.^3 + a44*t.^4 + a45*t.^5;
v4 = a41+2*a42*t+3*a43*t.^2 + 4*a44*t.^3 + 5*a45*t.^4;
a4 = 2*a42+6*a43*t + 12*a44*t.^2 + 20*a45*t.^3;
%第5个关节的角度、角速度、角加速度
q5 = a50+a51*t+a52*t.^2+a53*t.^3 + a54*t.^4 + a55*t.^5;
v5 = a51+2*a52*t+3*a53*t.^2 + 4*a54*t.^3 + 5*a55*t.^4;
a5 = 2*a52+6*a53*t + 12*a54*t.^2 + 20*a55*t.^3;
%第6个关节的角度、角速度、角加速度
q6 = a60+a61*t+a62*t.^2+a63*t.^3 + a64*t.^4 + a65*t.^5;
v6 = a61+2*a62*t+3*a63*t.^2 + 4*a64*t.^3 + 5*a65*t.^4;
a6 = 2*a62+6*a63*t + 12*a64*t.^2 + 20*a65*t.^3;

figure
subplot(2, 3, 1);
plot(t, q1,'LineWidth',1.5);
ylabel('角度/rad');
xlabel('t/s');
set(gca,'YLim',[min(q1)-(max(q1)-min(q1))/10 max(q1)+(max(q1)-min(q1))/10]);
axis square
grid on;
title('第1关节角度轨迹')

subplot(2, 3, 2);
plot(t, q2,'LineWidth',1.5);
ylabel('角度/rad');
xlabel('t/s');
set(gca,'YLim',[min(q2)-(max(q2)-min(q2))/10 max(q2)+(max(q2)-min(q2))/10]);
axis square
grid on;
title('第2关节角度轨迹')

subplot(2, 3, 3);
plot(t, q3,'LineWidth',1.5);
ylabel('角度/rad');
xlabel('t/s');
set(gca,'YLim',[min(q3)-(max(q3)-min(q3))/10 max(q3)+(max(q3)-min(q3))/10]);
axis square
grid on;
title('第3关节角度轨迹')

subplot(2, 3, 4);
plot(t, q4,'LineWidth',1.5);
ylabel('角度/rad');
xlabel('t/s');
set(gca,'YLim',[min(q4)-(max(q4)-min(q4))/10 max(q4)+(max(q4)-min(q4))/10]);
axis square
grid on;
title('第4关节角度轨迹')

subplot(2, 3, 5);
plot(t, q5,'LineWidth',1.5);
ylabel('角度/rad');
xlabel('t/s');
set(gca,'YLim',[min(q5)-(max(q5)-min(q5))/10 max(q5)+(max(q5)-min(q5))/10]);
axis square
grid on;
title('第5关节角度轨迹')

subplot(2, 3, 6);
plot(t, q6,'LineWidth',1.5);
ylabel('角度/rad');
xlabel('t/s');
set(gca,'YLim',[min(q6)-(max(q6)-min(q6))/10 max(q6)+(max(q6)-min(q6))/10]);
axis square
grid on;
title('第6关节角度轨迹')

figure
subplot(2, 3, 1);
plot(t, v1,'LineWidth',1.5);
ylabel('角速度/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v1)-(max(v1)-min(v1))/10 max(v1)+(max(v1)-min(v1))/10]);
axis square
grid on;
title('第1关节角速度轨迹')

subplot(2, 3, 2);
plot(t, v2,'LineWidth',1.5);
ylabel('角速度/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v2)-(max(v2)-min(v2))/10 max(v2)+(max(v2)-min(v2))/10]);
axis square
grid on;
title('第2关节角速度轨迹')

subplot(2, 3, 3);
plot(t, v3,'LineWidth',1.5);
ylabel('角速度/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v3)-(max(v3)-min(v3))/10 max(v3)+(max(v3)-min(v3))/10]);
axis square
grid on;
title('第3关节角速度轨迹')

subplot(2, 3, 4);
plot(t, v4,'LineWidth',1.5);
ylabel('角速度/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v4)-(max(v4)-min(v4))/10 max(v4)+(max(v4)-min(v4))/10]);
axis square
grid on;
title('第4关节角速度轨迹')

subplot(2, 3, 5);
plot(t, v5,'LineWidth',1.5);
ylabel('角速度/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v5)-(max(v5)-min(v5))/10 max(v5)+(max(v5)-min(v5))/10]);
axis square
grid on;
title('第5关节角速度轨迹')

subplot(2, 3, 6);
plot(t, v6,'LineWidth',1.5);
ylabel('角速度/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v6)-(max(v6)-min(v6))/10 max(v6)+(max(v6)-min(v6))/10]);
axis square
grid on;
title('第6关节角速度轨迹')

figure
subplot(2, 3, 1);
plot(t, a1,'LineWidth',1.5);
ylabel('角加速度{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a1)-(max(a1)-min(a1))/10 max(a1)+(max(a1)-min(a1))/10]);
axis square
grid on;
title('第1关节角加速度轨迹')

subplot(2, 3, 2);
plot(t, a2,'LineWidth',1.5);
ylabel('角加速度{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a2)-(max(a2)-min(a2))/10 max(a2)+(max(a2)-min(a2))/10]);
axis square
grid on;
title('第2关节角加速度轨迹')

subplot(2, 3, 3);
plot(t, a3,'LineWidth',1.5);
ylabel('角加速度{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a3)-(max(a3)-min(a3))/10 max(a3)+(max(a3)-min(a3))/10]);
axis square
grid on;
title('第3关节角加速度轨迹')

subplot(2, 3, 4);
plot(t, a4,'LineWidth',1.5);
ylabel('角加速度{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a4)-(max(a4)-min(a4))/10 max(a4)+(max(a4)-min(a4))/10]);
axis square
grid on;
title('第4关节角加速度轨迹')

subplot(2, 3, 5);
plot(t, a5,'LineWidth',1.5);
ylabel('角加速度{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a5)-(max(a5)-min(a5))/10 max(a5)+(max(a5)-min(a5))/10]);
axis square
grid on;
title('第5关节角加速度轨迹')

subplot(2, 3, 6);
plot(t, a6,'LineWidth',1.5);
ylabel('角加速度{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a6)-(max(a6)-min(a6))/10 max(a6)+(max(a6)-min(a6))/10]);
axis square
grid on;
title('第6关节角加速度轨迹')
