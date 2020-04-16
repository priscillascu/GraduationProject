%本程序用于给论文中的轨迹规划章节进行数据测试
clear all

t = 0:0.1:10;   %共1001个采样点
T1 = myFKSolver([0; 0; 0; 0; 0; 0]);   %起点位姿矩阵
T2 = [T1(1:4, 1:3), [0.7331; 0.2404; 0.8982; 1]];   %终点位姿矩阵

% N = 100;
% dx = -0.2/100;
% dy = 0.2/100;
% dz = -0.2/100;
% 
% x1 = [];
% y1 = [];
% z1 = [];
% for i = 1 : 100
%     x1(i) = 0.9331+i*dx;
%     y1(i) = 0.0404+i*dy;
%     z1(i) = 1.0982+i*dz;
% end
% 
% figure
% subplot(1, 3, 1)
% plot(t, x1,'LineWidth',1.5);
% xlabel('t');
% ylabel('x');
% grid on;
% title('x轴方向位置变化')
% 
% subplot(1, 3, 2)
% plot(t, y1,'LineWidth',1.5);
% xlabel('t');
% ylabel('y');
% grid on;
% title('y轴方向位置变化')
% 
% subplot(1, 3, 3)
% plot(t, z1,'LineWidth',1.5);
% xlabel('t');
% ylabel('z');
% grid on;
% title('z轴方向位置变化')
% 
% figure
% plot3(x1,y1,z1,'LineWidth',1.5);
% hold on
% plot3(0.9331, 0.0404, 1.0982, 'ro','MarkerFaceColor','r')
% text(0.9331, 0.0404, 1.0782, 'P0');
% hold on
% plot3(0.7331,0.2404,0.8982, 'ro','MarkerFaceColor','r')
% text(0.7331, 0.2404, 0.8782, 'P1');
% 
% xlabel('x');
% ylabel('y');
% zlabel('z');
% grid on;
% title('笛卡尔空间轨迹')

[x, dx, ddx] = lspb(0.9331, 0.7331, 101, 0.0025);
[y, dy, ddy] = lspb(0.0404, 0.2404, 101, 0.0025);
[z, dz, ddz] = lspb(1.0982, 0.8982, 101, 0.0025);

figure
subplot(1, 3, 1)
plot(t, x,'LineWidth',1.5);
xlabel('t/s');
ylabel('x/m');
set(gca,'YLim',[min(x)-(max(x)-min(x))/10 max(x)+(max(x)-min(x))/10]);
axis square
grid on;
title('x轴方向位移曲线')

subplot(1, 3, 2)
plot(t, y,'LineWidth',1.5);
xlabel('t/s');
ylabel('y/m');
set(gca,'YLim',[min(y)-(max(y)-min(y))/10 max(y)+(max(y)-min(y))/10]);
axis square
grid on;
title('y轴方向位移曲线')

subplot(1, 3, 3)
plot(t, z,'LineWidth',1.5);
xlabel('t/s');
ylabel('z/m');
set(gca,'YLim',[min(z)-(max(z)-min(z))/10 max(z)+(max(z)-min(z))/10]);
axis square
grid on;
title('z轴方向位移曲线')

figure
subplot(1, 3, 1)
plot(t, dx,'LineWidth',1.5);
set(gca,'YLim',[min(dx)-(max(dx)-min(dx))/10 max(dx)+(max(dx)-min(dx))/10]);
axis square
xlabel('t/s');
ylabel('{v_x/m/s}');
grid on;
title('x轴方向速度曲线')

subplot(1, 3, 2)
plot(t, dy,'LineWidth',1.5);
set(gca,'YLim',[min(dy)-(max(dy)-min(dy))/10 max(dy)+(max(dy)-min(dy))/10]);
axis square
xlabel('t/s');
ylabel('{v_y/m/s}');
grid on;
title('y轴方向速度曲线')

subplot(1, 3, 3)
plot(t, dz,'LineWidth',1.5);
set(gca,'YLim',[min(dz)-(max(dz)-min(dz))/10 max(dz)+(max(dz)-min(dz))/10]);
axis square
xlabel('t/s');
ylabel('{v_z/m/s}');
grid on;
title('z轴方向速度曲线')

figure
subplot(1, 3, 1)
plot(t, ddx,'LineWidth',1.5);
xlabel('t/s');
ylabel('{a_x/m/s^2}');
set(gca,'YLim',[min(ddx)-(max(ddx)-min(ddx))/10 max(ddx)+(max(ddx)-min(ddx))/10]);
axis square
grid on;
title('x轴方向加速度曲线')

subplot(1, 3, 2)
plot(t, ddy,'LineWidth',1.5);
xlabel('t/s');
ylabel('{a_y/m/s^2}');
set(gca,'YLim',[min(ddy)-(max(ddy)-min(ddy))/10 max(ddy)+(max(ddy)-min(ddy))/10]);
axis square
grid on;
title('y轴方向加速度曲线')

subplot(1, 3, 3)
plot(t, ddz,'LineWidth',1.5);
xlabel('t/s');
ylabel('{a_z/m/s^2}');
set(gca,'YLim',[min(ddz)-(max(ddz)-min(ddz))/10 max(ddz)+(max(ddz)-min(ddz))/10]);
axis square
grid on;
title('z轴方向加速度曲线')
