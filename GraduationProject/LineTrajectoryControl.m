clear all;

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

vx = v*(-0.2)/0.3464;
vy = v*(0.2)/0.3464;
vz = v*(-0.2)/0.3464;

%%
%使用指数趋近律的滑模控制
% dth = [0; 0; 0; 0; 0; 0];
% th = [0; 0; 0; 0; 0; 0];
% 
% k = 0.1;
% ita = 0.01;
% c = 5;
% alpha = 2;
% 
% x1 = [0; 0; 0; 0; 0; 0];
% xc = [0];
% yc = [0];
% zc = [0];
% e = [0; 0; 0; 0; 0; 0];
% for i = 1 : length(T)
%     %位置和速度给定
%     xd = [lx(i); ly(i); lz(i); 0; 0; 0];
%     dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
%     %解出当前位置
%     Tc = myFKSolver(th(:, i));
%     xc(i) = Tc(1, 4);
%     yc(i) = Tc(2, 4);
%     zc(i) = Tc(3, 4);
%     x = [xc(i); yc(i); zc(i); 0; 0; 0];
%     %求解当前角度下的雅可比矩阵
%     Jac = JacobianCross(th(:, i));
%     %误差
%     e(:, i) = x - xd;
%     s = c*e(i);
%     
%     dth(:, i) = inv(Jac)*(dxd - ita*tanh(s/0.1) - k*abs(s).^alpha*tanh(s/0.1));
%     th(:, i + 1) = th(:, i) + dth(:, i)*0.1;
% end

%%
%基于非奇异终端滑模的轨迹规划
% dth = [0; 0; 0; 0; 0; 0];
% 
% 
% x1 = [0; 0; 0; 0; 0; 0];
% xc = [0];
% yc = [0];
% zc = [0];
% intx = [0; 0; 0; 0; 0; 0];
% e = [0; 0; 0; 0; 0; 0];
% de = [0; 0; 0; 0; 0; 0];
% intxd = [0; 0; 0; 0; 0; 0];
% 
% q = 3;
% p = 5;
% 
% %针对初始角度为0的时候
% th = [0; 0; 0; 0; 0; 0]; %初始角度
% beta = 30;
% xite = 2;
% 
% %针对初始角度不为0的时候
% % beta = 8.5;
% % xite = 0.001;
% % th = [1; 1; 1; 1; 1; 1];  %初始角度
% 
% for i = 1 : length(T)
%     %位置和速度给定 位置积分给定、位置积分给定
%     xd = [lx(i); ly(i); lz(i); 0; 0; 0];
%     dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
%     intxd = intxd + xd;
%     
%     %解出当前位置和位置积分
%     Tc = myFKSolver(th(:, i));
%     xc(i) = Tc(1, 4);
%     yc(i) = Tc(2, 4);
%     zc(i) = Tc(3, 4);
%     x = [xc(i); yc(i); zc(i); 0; 0; 0];
%     intx = intx + x;
%     
%     e(:, i) = intx - intxd;
%     de(:, i) = x - xd;
%     
%     Jac = JacobianCross(th(:, i));
%     T1 = abs(de(:, i)).^(p/q).*tanh(de(:, i)/0.1);
%     T2 = abs(de(:, i)).^(2-p/q).*tanh(de(:, i)/0.1);   %若直接用x2^(2-p/q)则会出现计算错误，因为2-p/q可能为开根，x2可能小于0
%     s = e(:, i) + 1/beta*T1;
%     
%     dth(:, i) = -inv(Jac)*((beta*q/p)*T2 + xite*tanh(s/0.1));
% 
%     th(:, i + 1) = th(:, i) + dth(:, i)*0.1;
% end

%%
%基于固定时间的轨迹规划
%基于固定时间的轨迹规划
dth = [0; 0; 0; 0; 0; 0];
x1 = [0; 0; 0; 0; 0; 0];
xc = [0];
yc = [0];
zc = [0];
intx = [0; 0; 0; 0; 0; 0];
e = [0; 0; 0; 0; 0; 0];
de = [0; 0; 0; 0; 0; 0];
intxd = [0; 0; 0; 0; 0; 0];
x_before = [0; 0; 0; 0; 0; 0];
x = [0.9331; 0.0404; 1.0982; 0; 0; 0];  %初始位置

%针对初始角度为0的时候
th = [0; 0; 0; 0; 0; 0]; %初始角度
%0时刻的误差值e0
e0 = [0; 0; 0; 0; 0; 0];
de0 = [0; 0; 0; 0; 0; 0];
dde0 = [0; 0; 0; 0; 0; 0];
c1 = 1.3*[1; 1; 1; 1; 1; 1];
C = [c1];
lamda = 1;
% 
%针对初始角度不为0的时候
% th = [1; 1; 1; 1; 1; 1]; %初始角度
% %0时刻的误差值e0
% e0 = [-0.7032; 0.5026; -1.1838; 0; 0; 0];
% de0 = [0; 0; 0; 0; 0; 0];
% dde0 = [0; 0; 0; 0; 0; 0];
% c1 = 1.1*[0.8; 0.8; 0.8; 1; 1; 1];
% C = [c1];
% K = 1.7;


%P算子中的系数a
ctime = 1;
a00 = -3;
a10 = 2;
a01 = -2;
a11 = 1;

for i = 1 : length(T)
    %位置和速度给定 位置积分给定、位置积分给定
    xd = [lx(i); ly(i); lz(i); 0; 0; 0];
    dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
    intxd = intxd + xd;
    
    %解出当前位置和位置积分
    Tc = myFKSolver(th(:, i));
    xc(i) = Tc(1, 4);
    yc(i) = Tc(2, 4);
    zc(i) = Tc(3, 4);
    x_before = x;
    x = [xc(i); yc(i); zc(i); 0; 0; 0];
    intx = intx + x;
    dx = (x - x_before)/10;
    
    if(i >= ctime*10)
        p = [0; 0; 0; 0; 0; 0];
        dp = [0; 0; 0; 0; 0; 0];
    else   
        A1 = a00/(ctime^2)*e0 + a01/(ctime)*de0;  %t^2的系数项
        B1 = a10/(ctime^3)*e0 + a11/(ctime^2)*de0;  %t^3的系数项
        p = e0 + de0*i/10 + A1*(i/10)^2 + B1*(i/10)^3;
        dp = de0 + 2*A1*(i/10) + 3*B1*(i/10)^2;
    end
    
    e(:, i) = x - xd;
    de(:, i) = dx - dxd;
    
    E = [e(:, i)];
    P = [p];
    s = C.*E - C.*P;
    
    Jac = JacobianCross(th(:, i));
    
%     dth(:, i) = -Jac'*inv(Jac*Jac' + lamda^2 .* diag([1 1 1 1 1 1], 0))*(- dxd - dp + C.*tanh(s/0.1));
dth(:, i) = -inv(Jac)*(- dxd - dp + C.*tanh(s/0.1));

    th(:, i + 1) = th(:, i) + dth(:, i)*0.1;
end


%%
%使用迭代法求解关节角度
% lamda = 1;
% dth = [0; 0; 0; 0; 0; 0];
% th = [0; 0; 0; 0; 0; 0];
% for i = 1: length(T)
%     xd = [lx(i); ly(i); lz(i); 0; 0; 0];
%     dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
%     for j = 1: 1: 1000
%         Jac = JacobianCross(th(:, i));
%         Tc = myFKSolver(th(:, i));
%         xc(i) = Tc(1, 4);
%         yc(i) = Tc(2, 4);
%         zc(i) = Tc(3, 4);
%         dx = xd - [xc(i); yc(i); zc(i); 0; 0; 0]
%         dth(: , i) = Jac'*inv(Jac*Jac' + lamda^2 .* diag([1 1 1 1 1 1], 0))*dx;
%         th(:, i+1) = th(:, i) + dth(: , i)*0.1;
%         if norm(dx) < 0.0001
%             disp('求解完成')
%             break
%         else
%             disp('求解中')
%         end
%     end
% end

%%
%作图
figure
subplot(1, 3, 1)
plot(T, lx,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, xc,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('位移','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T) max(T)]);
axis square
grid on;
title('x轴方向位移跟踪曲线','FontName','黑体','FontSize',12)


subplot(1, 3, 2)
plot(T, ly,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, yc,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('位移','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T) max(T)]);
axis square
grid on;
title('y轴方向位移跟踪曲线','FontName','黑体','FontSize',12)

subplot(1, 3, 3)
plot(T, lz,'--','LineWidth',1.5,'color','k');
hold on
plot(T, zc, 'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('位移','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T) max(T)]);
axis square
grid on;
title('z轴方向位移跟踪曲线','FontName','黑体','FontSize',12)
% 
% figure
% subplot(2, 3, 1);
% plot(T, dth(1, :));
% subplot(2, 3, 2);
% plot(T, dth(2, :));
% subplot(2, 3, 3);
% plot(T, dth(3, :));
% subplot(2, 3, 4);
% plot(T, dth(4, :));
% subplot(2, 3, 5);
% plot(T, dth(5, :));
% subplot(2, 3, 6);
% plot(T, dth(6, :));
% 
% figure
% subplot(1, 3, 1);
% plot(T, e(1, :),'LineWidth',1.5,'color','k');
% xlabel('时间','FontName','黑体','FontSize',12);
% ylabel('误差值','FontName','黑体','FontSize',12);
% set(gca,'XLim',[min(T) max(T)]);
% axis square
% grid on;
% title('x轴方向跟踪误差','FontName','黑体','FontSize',12)
% 
% subplot(1, 3, 2);
% plot(T, e(2, :),'LineWidth',1.5,'color','k');
% xlabel('时间','FontName','黑体','FontSize',12);
% ylabel('误差值','FontName','黑体','FontSize',12);
% set(gca,'XLim',[min(T) max(T)]);
% axis square
% grid on;
% title('y轴方向跟踪误差','FontName','黑体','FontSize',12)
% 
% subplot(1, 3, 3);
% plot(T, e(3, :),'LineWidth',1.5,'color','k');
% xlabel('时间','FontName','黑体','FontSize',12);
% ylabel('误差值','FontName','黑体','FontSize',12);
% set(gca,'XLim',[min(T) max(T)]);
% axis square
% grid on;
% title('z轴方向跟踪误差','FontName','黑体','FontSize',12)

T2 = 0:0.1:10.1;
figure
subplot(2, 3, 1);
plot(T2, th(1, :),'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度值','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T2) max(T2)]);
grid on;
title('第1关节角度变化曲线','FontName','黑体','FontSize',12)


subplot(2, 3, 2);
plot(T2, th(2, :),'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度值','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T2) max(T2)]);
grid on;
title('第2关节角度变化曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 3);
plot(T2, th(3, :),'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度值','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T2) max(T2)]);
grid on;
title('第3关节角度变化曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 4);
plot(T2, th(4, :),'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度值','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T2) max(T2)]);
grid on;
title('第4关节角度变化曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 5);
plot(T2, th(5, :),'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度值','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T2) max(T2)]);
grid on;
title('第5关节角度变化曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 6);
plot(T2, th(6, :),'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度值','FontName','黑体','FontSize',12);
set(gca,'XLim',[min(T2) max(T2)]);
grid on;
title('第6关节角度变化曲线','FontName','黑体','FontSize',12)