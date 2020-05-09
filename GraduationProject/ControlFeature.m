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
legend('��������', 'ʵ������');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ�','FontName','����','FontSize',12);
axis square
grid on;
title('��1�ؽڹ켣��������','FontName','����','FontSize',12)

subplot(2, 3, 2)
plot(T, xd2g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q2g,'LineWidth',1.5,'color','k');
legend('��������', 'ʵ������');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ�','FontName','����','FontSize',12);
axis square
grid on;
title('��2�ؽڹ켣��������','FontName','����','FontSize',12)

subplot(2, 3, 3)
plot(T, xd3g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q3g,'LineWidth',1.5,'color','k');
legend('��������', 'ʵ������');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ�','FontName','����','FontSize',12);
axis square
grid on;
title('��3�ؽڹ켣��������','FontName','����','FontSize',12)

subplot(2, 3, 4)
plot(T, xd4g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q4g,'LineWidth',1.5,'color','k');
legend('��������', 'ʵ������');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ�','FontName','����','FontSize',12);
axis square
grid on;
title('��4�ؽڹ켣��������','FontName','����','FontSize',12)

subplot(2, 3, 5)
plot(T, xd5g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q5g,'LineWidth',1.5,'color','k');
legend('��������', 'ʵ������');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ�','FontName','����','FontSize',12);
axis square
grid on;
title('��5�ؽڹ켣��������','FontName','����','FontSize',12)

subplot(2, 3, 6)
plot(T, xd6g,'--', 'LineWidth',1.5,'color','k');
hold on
plot(T, q6g,'LineWidth',1.5,'color','k');
legend('��������', 'ʵ������');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ�','FontName','����','FontSize',12);
axis square
grid on;
title('��6�ؽڹ켣��������','FontName','����','FontSize',12)

%���
figure
subplot(2, 3, 1)
plot(T, e1g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ����','FontName','����','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('��1�ؽڽǶȸ������','FontName','����','FontSize',12)

subplot(2, 3, 2)
plot(T, e2g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ����','FontName','����','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('��2�ؽڽǶȸ������','FontName','����','FontSize',12)

subplot(2, 3, 3)
plot(T, e3g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ����','FontName','����','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('��3�ؽڽǶȸ������','FontName','����','FontSize',12)

subplot(2, 3, 4)
plot(T, e4g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ����','FontName','����','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('��4�ؽڽǶȸ������','FontName','����','FontSize',12)

subplot(2, 3, 5)
plot(T, e5g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ����','FontName','����','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('��5�ؽڽǶȸ������','FontName','����','FontSize',12)

subplot(2, 3, 6)
plot(T, e6g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('�Ƕ����','FontName','����','FontSize',12);
set(gca,'YLim',[-0.1, 0.1]);
axis square
grid on;
title('��6�ؽڽǶȸ������','FontName','����','FontSize',12)

%�������
figure
subplot(2, 3, 1)
plot(T, u1g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('��������','FontName','����','FontSize',12);
axis square
grid on;
title('��1�ؽڿ�����������','FontName','����','FontSize',12)

subplot(2, 3, 2)
plot(T, u2g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('��������','FontName','����','FontSize',12);
axis square
grid on;
title('��2�ؽڿ�����������','FontName','����','FontSize',12)

subplot(2, 3, 3)
plot(T, u3g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('��������','FontName','����','FontSize',12);
axis square
grid on;
title('��3�ؽڿ�����������','FontName','����','FontSize',12)

subplot(2, 3, 4)
plot(T, u4g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('��������','FontName','����','FontSize',12);
axis square
grid on;
title('��4�ؽڿ�����������','FontName','����','FontSize',12)

subplot(2, 3, 5)
plot(T, u5g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('��������','FontName','����','FontSize',12);
axis square
grid on;
title('��5�ؽڿ�����������','FontName','����','FontSize',12)

subplot(2, 3, 6)
plot(T, u6g,'-', 'LineWidth',1.5,'color','k');
xlabel('ʱ��','FontName','����','FontSize',12);
ylabel('��������','FontName','����','FontSize',12);
axis square
grid on;
title('��6�ؽڿ�����������','FontName','����','FontSize',12)