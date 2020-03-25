%������Ϊ��CRP14��е�۹滮�ѿ����ռ仭Բ�켣
%��û�е����״���ӣ���ͳDH��ģ�����ã�׼������ֱ��������任�����������˶�ѧ
%���ñ任���������������˶�ѧ
clear all
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
Tx = [0, 0.1949, 1.0938e-04, 0.2000, 0, -4.7162e-04];
Ty = [0, -0.0951, -0.6137, 0.2750, 0.0320, 0.0981];
Tz = [0, 0.2850, 0.0030, 0.1105, 0.3650, 0.0540];
th = [1, 1, 1-pi/2, 1, 1, 1];
T0 = 1;
for i = 1:6
    T{i} = transl(Tx(i), Ty(i), Tz(i))*trotx(alpha(i))*trotz(th(i));  %��ƽ������ת
end
trplot(T{1}, 'frame', '1', 'color', 'b');
hold on
trplot(T{1}*T{2}, 'frame', '2', 'color', 'g');
hold on
trplot(T{1}*T{2}*T{3}, 'frame', '3', 'color', 'r');
hold on
trplot(T{1}*T{2}*T{3}*T{4}, 'frame', '4', 'color', 'c');
hold on
trplot(T{1}*T{2}*T{3}*T{4}*T{5}, 'frame', '5');
hold on
trplot(T{1}*T{2}*T{3}*T{4}*T{5}*T{6}, 'frame', '6', 'color', 'm');

%���˶�ѧ���

