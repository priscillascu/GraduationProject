clear;
clc;
%��һ���ؽڵĳ�ʼ����ֹ�Ƕ�
q01 = 0;
qf1 = 0.3131; 
%�ڶ����ؽڵĳ�ʼ����ֹ�Ƕ�
q02 = 0;
qf2 = -0.3181; 
%�������ؽڵĳ�ʼ����ֹ�Ƕ�
q03 = 0;
qf3 = 0.5993; 
%���ĸ��ؽڵĳ�ʼ����ֹ�Ƕ�
q04 = 0;
qf4 = 0.8623; 
%������ؽڵĳ�ʼ����ֹ�Ƕ�
q05 = 0;
qf5 = -0.4178; 
%�������ؽڵĳ�ʼ����ֹ�Ƕ�
q06 = 0;
qf6 = -0.8176; 

%ָ����ֹʱ��
t0 = 0;
tf = 10;
%ָ����ֹ�ٶ�
v0 = 0;
vf = 0;

%�����1���ؽڵ����ζ���ʽϵ��
a10 = q01;
a11 = v0;
a12=(3/(tf)^2)*(qf1-q01)-(1/tf)*(2*v0+vf);
a13=(2/(tf)^3)*(q01-qf1)+(1/tf^2)*(v0+vf);
%�����2���ؽڵ����ζ���ʽϵ��
a20 = q02;
a21 = v0;
a22=(3/(tf)^2)*(qf2-q02)-(1/tf)*(2*v0+vf);
a23=(2/(tf)^3)*(q02-qf2)+(1/tf^2)*(v0+vf);
%�����3���ؽڵ����ζ���ʽϵ��
a30 = q03;
a31 = v0;
a32=(3/(tf)^2)*(qf3-q03)-(1/tf)*(2*v0+vf);
a33=(2/(tf)^3)*(q03-qf3)+(1/tf^2)*(v0+vf);
%�����4���ؽڵ����ζ���ʽϵ��
a40 = q04;
a41 = v0;
a42=(3/(tf)^2)*(qf4-q04)-(1/tf)*(2*v0+vf);
a43=(2/(tf)^3)*(q04-qf4)+(1/tf^2)*(v0+vf);
%�����5���ؽڵ����ζ���ʽϵ��
a50 = q05;
a51 = v0;
a52=(3/(tf)^2)*(qf5-q05)-(1/tf)*(2*v0+vf);
a53=(2/(tf)^3)*(q05-qf5)+(1/tf^2)*(v0+vf);
%�����6���ؽڵ����ζ���ʽϵ��
a60 = q06;
a61 = v0;
a62=(3/(tf)^2)*(qf6-q06)-(1/tf)*(2*v0+vf);
a63=(2/(tf)^3)*(q06-qf6)+(1/tf^2)*(v0+vf);

t = t0:0.01:tf;
%��1���ؽڵĽǶȡ����ٶȡ��Ǽ��ٶ�
q1 = a10+a11*t+a12*t.^2+a13*t.^3;
v1 = a11+2*a12*t+3*a13*t.^2;
a1 = 2*a12+6*a13*t;
%��2���ؽڵĽǶȡ����ٶȡ��Ǽ��ٶ�
q2 = a20+a21*t+a22*t.^2+a23*t.^3;
v2 = a21+2*a22*t+3*a23*t.^2;
a2 = 2*a22+6*a23*t;
%��3���ؽڵĽǶȡ����ٶȡ��Ǽ��ٶ�
q3 = a30+a31*t+a32*t.^2+a33*t.^3;
v3 = a31+2*a32*t+3*a33*t.^2;
a3 = 2*a32+6*a33*t;
%��4���ؽڵĽǶȡ����ٶȡ��Ǽ��ٶ�
q4 = a40+a41*t+a42*t.^2+a43*t.^3;
v4 = a41+2*a42*t+3*a43*t.^2;
a4 = 2*a42+6*a43*t;
%��5���ؽڵĽǶȡ����ٶȡ��Ǽ��ٶ�
q5 = a50+a51*t+a52*t.^2+a53*t.^3;
v5 = a51+2*a52*t+3*a53*t.^2;
a5 = 2*a52+6*a53*t;
%��6���ؽڵĽǶȡ����ٶȡ��Ǽ��ٶ�
q6 = a60+a61*t+a62*t.^2+a63*t.^3;
v6 = a61+2*a62*t+3*a63*t.^2;
a6 = 2*a62+6*a63*t;

figure
subplot(2, 3, 1);
plot(t, q1,'LineWidth',1.5);
ylabel('�Ƕ�/rad');
xlabel('t/s');
set(gca,'YLim',[min(q1)-(max(q1)-min(q1))/10 max(q1)+(max(q1)-min(q1))/10]);
axis square
grid on;
title('��1�ؽڽǶȹ켣')

subplot(2, 3, 2);
plot(t, q2,'LineWidth',1.5);
ylabel('�Ƕ�/rad');
xlabel('t/s');
set(gca,'YLim',[min(q2)-(max(q2)-min(q2))/10 max(q2)+(max(q2)-min(q2))/10]);
axis square
grid on;
title('��2�ؽڽǶȹ켣')

subplot(2, 3, 3);
plot(t, q3,'LineWidth',1.5);
ylabel('�Ƕ�/rad');
xlabel('t/s');
set(gca,'YLim',[min(q3)-(max(q3)-min(q3))/10 max(q3)+(max(q3)-min(q3))/10]);
axis square
grid on;
title('��3�ؽڽǶȹ켣')

subplot(2, 3, 4);
plot(t, q4,'LineWidth',1.5);
ylabel('�Ƕ�/rad');
xlabel('t/s');
set(gca,'YLim',[min(q4)-(max(q4)-min(q4))/10 max(q4)+(max(q4)-min(q4))/10]);
axis square
grid on;
title('��4�ؽڽǶȹ켣')

subplot(2, 3, 5);
plot(t, q5,'LineWidth',1.5);
ylabel('�Ƕ�/rad');
xlabel('t/s');
set(gca,'YLim',[min(q5)-(max(q5)-min(q5))/10 max(q5)+(max(q5)-min(q5))/10]);
axis square
grid on;
title('��5�ؽڽǶȹ켣')

subplot(2, 3, 6);
plot(t, q6,'LineWidth',1.5);
ylabel('�Ƕ�/rad');
xlabel('t/s');
set(gca,'YLim',[min(q6)-(max(q6)-min(q6))/10 max(q6)+(max(q6)-min(q6))/10]);
axis square
grid on;
title('��6�ؽڽǶȹ켣')

figure
subplot(2, 3, 1);
plot(t, v1,'LineWidth',1.5);
ylabel('���ٶ�/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v1)-(max(v1)-min(v1))/10 max(v1)+(max(v1)-min(v1))/10]);
axis square
grid on;
title('��1�ؽڽ��ٶȹ켣')

subplot(2, 3, 2);
plot(t, v2,'LineWidth',1.5);
ylabel('���ٶ�/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v2)-(max(v2)-min(v2))/10 max(v2)+(max(v2)-min(v2))/10]);
axis square
grid on;
title('��2�ؽڽ��ٶȹ켣')

subplot(2, 3, 3);
plot(t, v3,'LineWidth',1.5);
ylabel('���ٶ�/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v3)-(max(v3)-min(v3))/10 max(v3)+(max(v3)-min(v3))/10]);
axis square
grid on;
title('��3�ؽڽ��ٶȹ켣')

subplot(2, 3, 4);
plot(t, v4,'LineWidth',1.5);
ylabel('���ٶ�/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v4)-(max(v4)-min(v4))/10 max(v4)+(max(v4)-min(v4))/10]);
axis square
grid on;
title('��4�ؽڽ��ٶȹ켣')

subplot(2, 3, 5);
plot(t, v5,'LineWidth',1.5);
ylabel('���ٶ�/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v5)-(max(v5)-min(v5))/10 max(v5)+(max(v5)-min(v5))/10]);
axis square
grid on;
title('��5�ؽڽ��ٶȹ켣')

subplot(2, 3, 6);
plot(t, v6,'LineWidth',1.5);
ylabel('���ٶ�/rad/s');
xlabel('t/s');
set(gca,'YLim',[min(v6)-(max(v6)-min(v6))/10 max(v6)+(max(v6)-min(v6))/10]);
axis square
grid on;
title('��6�ؽڽ��ٶȹ켣')

figure
subplot(2, 3, 1);
plot(t, a1,'LineWidth',1.5);
ylabel('�Ǽ��ٶ�{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a1)-(max(a1)-min(a1))/10 max(a1)+(max(a1)-min(a1))/10]);
axis square
grid on;
title('��1�ؽڽǼ��ٶȹ켣')

subplot(2, 3, 2);
plot(t, a2,'LineWidth',1.5);
ylabel('�Ǽ��ٶ�{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a2)-(max(a2)-min(a2))/10 max(a2)+(max(a2)-min(a2))/10]);
axis square
grid on;
title('��2�ؽڽǼ��ٶȹ켣')

subplot(2, 3, 3);
plot(t, a3,'LineWidth',1.5);
ylabel('�Ǽ��ٶ�{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a3)-(max(a3)-min(a3))/10 max(a3)+(max(a3)-min(a3))/10]);
axis square
grid on;
title('��3�ؽڽǼ��ٶȹ켣')

subplot(2, 3, 4);
plot(t, a4,'LineWidth',1.5);
ylabel('�Ǽ��ٶ�{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a4)-(max(a4)-min(a4))/10 max(a4)+(max(a4)-min(a4))/10]);
axis square
grid on;
title('��4�ؽڽǼ��ٶȹ켣')

subplot(2, 3, 5);
plot(t, a5,'LineWidth',1.5);
ylabel('�Ǽ��ٶ�{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a5)-(max(a5)-min(a5))/10 max(a5)+(max(a5)-min(a5))/10]);
axis square
grid on;
title('��5�ؽڽǼ��ٶȹ켣')

subplot(2, 3, 6);
plot(t, a6,'LineWidth',1.5);
ylabel('�Ǽ��ٶ�{/rad/s^2}');
xlabel('t/s');
set(gca,'YLim',[min(a6)-(max(a6)-min(a6))/10 max(a6)+(max(a6)-min(a6))/10]);
axis square
grid on;
title('��6�ؽڽǼ��ٶȹ켣')
