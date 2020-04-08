%本程序为S型七段速度曲线轨迹规划
v0 = 0;
v1 = sqrt(0.2^2+0.2^2+0.2^2)/(16*10/7);  %设置第一个加速度拐点速度v1

dt = 10/7;
t1 = dt;
t2 = 2*dt;
t3 = 3*dt;
t4 = 4*dt;
t5 = 5*dt;
t6 = 6*dt;
t7 = 7*dt;
J = 2*v1/(t1^2);    %加加速度
amax = J*t1;   %加速度最大值
v2 = v1 + amax*(t2 - t1);   
vmax = v1 + v2;

v = [];
dv = [];
sum_v = [];
sum_v(1) = 0;
T = 0 : 0.1 : 10;

for i = 1:length(T)  %共101次循环，7段函数
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

%轨迹沿着坐标轴位移分量
lx = 0.9331+sum_v*(-0.2)/0.3464;
ly = 0.0404+sum_v*(0.2)/0.3464;
lz = 1.0982+sum_v*(-0.2)/0.3464;

figure
subplot(3, 1, 1)
plot(T, sum_v,'LineWidth',1.5);
xlabel('t');
ylabel('位移x');
grid on;
title('位移变化曲线')
subplot(3, 1, 2)
plot(T, v,'LineWidth',1.5);
xlabel('t');
ylabel('速度v');
grid on;
title('速度变化曲线')

subplot(3, 1, 3)
plot(T, dv,'LineWidth',1.5);
xlabel('t');
ylabel('加速度a');
grid on;
title('加速度变化曲线')

figure
subplot(1, 3, 1)
plot(T, lx,'LineWidth',1.5);
xlabel('t');
ylabel('x');
grid on;
title('x轴方向位置变化')

subplot(1, 3, 2)
plot(T, ly,'LineWidth',1.5);
xlabel('t');
ylabel('y');
grid on;
title('y轴方向位置变化')

subplot(1, 3, 3)
plot(T, lz,'LineWidth',1.5);
xlabel('t');
ylabel('z');
grid on;
title('z轴方向位置变化')

%轨迹点逆运动学解算
T1 = myFKSolver([0; 0; 0; 0; 0; 0]);   %起点位姿矩阵
T2 = [T1(1:4, 1:3), [0.7331; 0.2404; 0.8982; 1]];   %终点位姿矩阵
Ts = [];
qc = [];
qb = [0; 0; 0; 0; 0; 0];  %初始角度
for i = 1 : length(T)
    Ts(:, :, i) = transl(lx(i)-0.9331, ly(i)-0.0404, lz(i)-1.0982)*T1;
    disp('开始解算');
    qc(:, i) = myIKSolver(qb, Ts(:, :, i));   %求出每个位姿的逆运动学解
    qb = qc(:, i);  %修改每次迭代的初始角度，使其更加接近实际角度
    i
end

figure
subplot(2, 3, 1)
plot(T, qc(1, :),'LineWidth',1.5);
xlabel('t');
ylabel('q1');
grid on;
title('第1关节关节角变化')

subplot(2, 3, 2)
plot(T, qc(2, :),'LineWidth',1.5);
xlabel('t');
ylabel('q2');
grid on;
title('第2关节关节角变化')

subplot(2, 3, 3)
plot(T, qc(3, :),'LineWidth',1.5);
xlabel('t');
ylabel('q3');
grid on;
title('第3关节关节角变化')

subplot(2, 3, 4)
plot(T, qc(4, :),'LineWidth',1.5);
xlabel('t');
ylabel('q4');
grid on;
title('第4关节关节角变化')

subplot(2, 3, 5)
plot(T, qc(5, :),'LineWidth',1.5);
xlabel('t');
ylabel('q5');
grid on;
title('第5关节关节角变化')

subplot(2, 3, 6)
plot(T, qc(6, :),'LineWidth',1.5);
xlabel('t');
ylabel('q6');
grid on;
title('第6关节关节角变化')


