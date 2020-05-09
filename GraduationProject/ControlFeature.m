T = xd1.Time;
xd1g = xd1.Data;
xd2g = xd2.Data;
xd3g = xd3.Data;
xd4g = xd4.Data;
xd5g = xd5.Data;
xd6g = xd6.Data;

e1g = e1.Data;
e2g = e2.Data;
e3g = e3.Data;
e4g = e4.Data;
e5g = e5.Data;
e6g = e6.Data;

q1g = q1.Data;
q2g = q2.Data;
q3g = q3.Data;
q4g = q4.Data;
q5g = q5.Data;
q6g = q6.Data;

u1g = outu1.Data;
u2g = outu2.Data;
u3g = outu3.Data;
u4g = outu4.Data;
u5g = outu5.Data;
u6g = outu6.Data;

figure
subplot(2, 3, 1)
plot(T, xd1g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q1g,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度','FontName','黑体','FontSize',12);
axis square
grid on;
title('第1关节轨迹跟踪曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 2)
plot(T, xd2g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q2g,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度','FontName','黑体','FontSize',12);
axis square
grid on;
title('第2关节轨迹跟踪曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 3)
plot(T, xd3g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q3g,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度','FontName','黑体','FontSize',12);
axis square
grid on;
title('第3关节轨迹跟踪曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 4)
plot(T, xd4g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q4g,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度','FontName','黑体','FontSize',12);
axis square
grid on;
title('第4关节轨迹跟踪曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 5)
plot(T, xd5g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q5g,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度','FontName','黑体','FontSize',12);
axis square
grid on;
title('第5关节轨迹跟踪曲线','FontName','黑体','FontSize',12)

subplot(2, 3, 6)
plot(T, xd6g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q6g,'LineWidth',1.5,'color','k');
legend('给定曲线', '实际曲线');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度','FontName','黑体','FontSize',12);
axis square
grid on;
title('第6关节轨迹跟踪曲线','FontName','黑体','FontSize',12)

%误差
figure
subplot(2, 3, 1)
plot(T, e1g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度误差','FontName','黑体','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('第1关节角度跟踪误差','FontName','黑体','FontSize',12)

subplot(2, 3, 2)
plot(T, e2g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度误差','FontName','黑体','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('第2关节角度跟踪误差','FontName','黑体','FontSize',12)

subplot(2, 3, 3)
plot(T, e3g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度误差','FontName','黑体','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('第3关节角度跟踪误差','FontName','黑体','FontSize',12)

subplot(2, 3, 4)
plot(T, e4g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度误差','FontName','黑体','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('第4关节角度跟踪误差','FontName','黑体','FontSize',12)

subplot(2, 3, 5)
plot(T, e5g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度误差','FontName','黑体','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('第5关节角度跟踪误差','FontName','黑体','FontSize',12)

subplot(2, 3, 6)
plot(T, e6g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('角度误差','FontName','黑体','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('第6关节角度跟踪误差','FontName','黑体','FontSize',12)

%控制输出
figure
subplot(2, 3, 1)
plot(T, u1g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('控制输入','FontName','黑体','FontSize',12);
axis square
grid on;
title('第1关节控制力矩输入','FontName','黑体','FontSize',12)

subplot(2, 3, 2)
plot(T, u2g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('控制输入','FontName','黑体','FontSize',12);
axis square
grid on;
title('第2关节控制力矩输入','FontName','黑体','FontSize',12)

subplot(2, 3, 3)
plot(T, u3g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('控制输入','FontName','黑体','FontSize',12);
axis square
grid on;
title('第3关节控制力矩输入','FontName','黑体','FontSize',12)

subplot(2, 3, 4)
plot(T, u4g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('控制输入','FontName','黑体','FontSize',12);
axis square
grid on;
title('第4关节控制力矩输入','FontName','黑体','FontSize',12)

subplot(2, 3, 5)
plot(T, u5g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('控制输入','FontName','黑体','FontSize',12);
axis square
grid on;
title('第5关节控制力矩输入','FontName','黑体','FontSize',12)

subplot(2, 3, 6)
plot(T, u6g,'-', 'LineWidth',1.5,'color','k');
xlabel('时间','FontName','黑体','FontSize',12);
ylabel('控制输入','FontName','黑体','FontSize',12);
axis square
grid on;
title('第6关节控制力矩输入','FontName','黑体','FontSize',12)