%������ΪS���߶��ٶ����߹켣�滮
v0 = 0;
v1 = sqrt(0.2^2+0.2^2+0.2^2)/(16*10/7);  %���õ�һ�����ٶȹյ��ٶ�v1

dt = 10/7;
t1 = dt;
t2 = 2*dt;
t3 = 3*dt;
t4 = 4*dt;
t5 = 5*dt;
t6 = 6*dt;
t7 = 7*dt;
J = 2*v1/(t1^2);    %�Ӽ��ٶ�
amax = J*t1;   %���ٶ����ֵ
v2 = v1 + amax*(t2 - t1);   
vmax = v1 + v2;

v = [];
dv = [];
sum_v = [];
sum_v(1) = 0;
T = 0 : 0.1 : 10;

for i = 1:length(T)  %��101��ѭ����7�κ���
    t = i/10;
    if t < t1
        v(i) = 0.5*J*t^2;
        dv(i) = J*t;
    end
    if t < t2 & t >= t1
       v(i) = 0.5*J*t1^2 + amax*(t - t1);
       dv(i) = amax;
    end
    if t >= t2 & t < t3
       v(i) = vmax - 0.5*J*(t3 - t)^2;
       dv(i) = -J*(t - t3);
    end
    if t >= t3 & t < t4
      v(i) = vmax;
      dv(i) = 0;
    end
    if t >= t4 & t < t5
       v(i) = vmax - 0.5*J*(t - t4)^2;
       dv(i) = -J*(t - t4);
    end
     if t >= t5 & t < t6
        v(i) = vmax - 0.5*J*(t5 - t4)^2 - amax*(t - t5);
        dv(i) = -amax;
     end
     if t >= t6 & t <= t7
         v(i) = 0.5*J*(t7 - t)^2;
         dv(i) = J*(t - t7);
     end
     if t > t7
         v(i) = 0;
         dv(i) = 0;
     end
     sum_v(i) = sum(0.1*v);
end

%�켣����������λ�Ʒ���
lx = 0.9331+sum_v*(-0.2)/0.3464;
ly = 0.0404+sum_v*(0.2)/0.3464;
lz = 1.0982+sum_v*(-0.2)/0.3464;

vx = v*(-0.2)/0.3464;
vy = v*(0.2)/0.3464;
vz = v*(-0.2)/0.3464;

ax = dv*(-0.2)/0.3464;
ay = dv*(0.2)/0.3464;
az = dv*(-0.2)/0.3464;

figure
subplot(3, 1, 1)
plot(T, sum_v,'LineWidth',1.5);
xlabel('t');
ylabel('λ��x');
grid on;
title('λ�Ʊ仯����')
subplot(3, 1, 2)
plot(T, v,'LineWidth',1.5);
xlabel('t');
ylabel('�ٶ�v');
grid on;
title('�ٶȱ仯����')

subplot(3, 1, 3)
plot(T, dv,'LineWidth',1.5);
xlabel('t');
ylabel('���ٶ�a');
grid on;
title('���ٶȱ仯����')

figure
subplot(1, 3, 1)
plot(T, lx,'LineWidth',1.5);
xlabel('t/s');
ylabel('{x/m}');
set(gca,'YLim',[min(x)-(max(x)-min(x))/10 max(x)+(max(x)-min(x))/10]);
axis square
grid on;
title('x�᷽��λ�Ʊ仯')

subplot(1, 3, 2)
plot(T, ly,'LineWidth',1.5);
xlabel('t/s');
ylabel('{y/m}');
set(gca,'YLim',[min(y)-(max(y)-min(y))/10 max(y)+(max(y)-min(y))/10]);
axis square
grid on;
title('y�᷽��λ�Ʊ仯')

subplot(1, 3, 3)
plot(T, lz,'LineWidth',1.5);
xlabel('t/s');
ylabel('{z/m}');
set(gca,'YLim',[min(z)-(max(z)-min(z))/10 max(z)+(max(z)-min(z))/10]);
axis square
grid on;
title('z�᷽��λ�Ʊ仯')

figure
subplot(1, 3, 1)
plot(T, vx,'LineWidth',1.5);
xlabel('t/s');
ylabel('{v_x/m/s}');
set(gca,'YLim',[min(vx)-(max(vx)-min(vx))/10 max(vx)+(max(vx)-min(vx))/10]);
axis square
grid on;
title('x�᷽���ٶȱ仯')

subplot(1, 3, 2)
plot(T, vy,'LineWidth',1.5);
xlabel('t/s');
ylabel('{v_y/m/s}');
set(gca,'YLim',[min(vy)-(max(vy)-min(vy))/10 max(vy)+(max(vy)-min(vy))/10]);
axis square
grid on;
title('y�᷽���ٶȱ仯')

subplot(1, 3, 3)
plot(T, vz,'LineWidth',1.5);
xlabel('t/s');
ylabel('{v_z/m/s}');
set(gca,'YLim',[min(vz)-(max(vz)-min(vz))/10 max(vz)+(max(vz)-min(vz))/10]);
axis square
grid on;
title('z�᷽���ٶȱ仯')

figure
subplot(1, 3, 1)
plot(T, ax,'LineWidth',1.5);
xlabel('t/s');
ylabel('{a_x/m/s^2}');
set(gca,'YLim',[min(ax)-(max(ax)-min(ax))/10 max(ax)+(max(ax)-min(ax))/10]);
axis square
grid on;
title('x�᷽����ٶȱ仯')

subplot(1, 3, 2)
plot(T, ay,'LineWidth',1.5);
xlabel('t/s');
ylabel('{a_y/m/s^2}');
set(gca,'YLim',[min(ay)-(max(ay)-min(ay))/10 max(ay)+(max(ay)-min(ay))/10]);
axis square
grid on;
title('y�᷽����ٶȱ仯')

subplot(1, 3, 3)
plot(T, az,'LineWidth',1.5);
xlabel('t/s');
ylabel('{a_z/m/s^2}');
set(gca,'YLim',[min(az)-(max(az)-min(az))/10 max(az)+(max(az)-min(az))/10]);
axis square
grid on;
title('z�᷽����ٶȱ仯')

%�켣�����˶�ѧ����
T1 = myFKSolver([0; 0; 0; 0; 0; 0]);   %���λ�˾���
T2 = [T1(1:4, 1:3), [0.7331; 0.2404; 0.8982; 1]];   %�յ�λ�˾���
Ts = [];
qc = [];
qb = [0; 0; 0; 0; 0; 0];  %��ʼ�Ƕ�
for i = 1 : length(T)
    Ts(:, :, i) = transl(lx(i)-0.9331, ly(i)-0.0404, lz(i)-1.0982)*T1;
    disp('��ʼ����');
    qc(:, i) = myIKSolver(qb, Ts(:, :, i));   %���ÿ��λ�˵����˶�ѧ��
    qb = qc(:, i);  %�޸�ÿ�ε����ĳ�ʼ�Ƕȣ�ʹ����ӽӽ�ʵ�ʽǶ�
    i
end

figure
subplot(2, 3, 1)
plot(T, qc(1, :),'LineWidth',1.5);
xlabel('t/s');
ylabel('{q_1/rad}');
set(gca,'YLim',[min(qc(1, :))-(max(qc(1, :))-min(qc(1, :)))/10 max(qc(1, :))+(max(qc(1, :))-min(qc(1, :)))/10]);
axis square
grid on;
title('��1�ؽڹؽڽǹ켣����')

subplot(2, 3, 2)
plot(T, qc(2, :),'LineWidth',1.5);
xlabel('t/s');
ylabel('{q_2/rad}');
set(gca,'YLim',[min(qc(2, :))-(max(qc(2, :))-min(qc(2, :)))/10 max(qc(2, :))+(max(qc(2, :))-min(qc(2, :)))/10]);
axis square
grid on;
title('��2�ؽڹؽڽǹ켣����')

subplot(2, 3, 3)
plot(T, qc(3, :),'LineWidth',1.5);
xlabel('t/s');
ylabel('{q_3/rad}');
set(gca,'YLim',[min(qc(3, :))-(max(qc(3, :))-min(qc(3, :)))/10 max(qc(3, :))+(max(qc(3, :))-min(qc(3, :)))/10]);
axis square
grid on;
title('��3�ؽڹؽڽǹ켣����')

subplot(2, 3, 4)
plot(T, qc(4, :),'LineWidth',1.5);
xlabel('t/s');
ylabel('{q_4/rad}');
set(gca,'YLim',[min(qc(4, :))-(max(qc(4, :))-min(qc(4, :)))/10 max(qc(4, :))+(max(qc(4, :))-min(qc(4, :)))/10]);
axis square
grid on;
title('��4�ؽڹؽڽǹ켣����')

subplot(2, 3, 5)
plot(T, qc(5, :),'LineWidth',1.5);
xlabel('t/s');
ylabel('{q_5/rad}');
set(gca,'YLim',[min(qc(5, :))-(max(qc(5, :))-min(qc(5, :)))/10 max(qc(5, :))+(max(qc(5, :))-min(qc(5, :)))/10]);
axis square
grid on;
title('��5�ؽڹؽڽǹ켣����')

subplot(2, 3, 6)
plot(T, qc(6, :),'LineWidth',1.5);
xlabel('t/s');
ylabel('{q_6/rad}');
set(gca,'YLim',[min(qc(6, :))-(max(qc(6, :))-min(qc(6, :)))/10 max(qc(6, :))+(max(qc(6, :))-min(qc(6, :)))/10]);
axis square
grid on;
title('��6�ؽڹؽڽǹ켣����')


