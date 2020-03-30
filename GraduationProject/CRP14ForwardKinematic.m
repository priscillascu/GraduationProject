%������Ϊ��CRP14��е�۹滮�ѿ����ռ仭Բ�켣
%��û�е����״���ӣ���ͳDH��ģ�����ã�׼������ֱ��������任�����������˶�ѧ
%�켣�滮�����������˶�ѧ��ֱ�����ſɱȾ�����켣�ٶ�
%���������˶�ѧ���ʽ
clear all
Tx = [0, 0.1949, 1.0938e-04, 0.2000, 0, -4.7162e-04];
Ty = [0, -0.0951, -0.6137, 0.2750, 0.0320, 0.0981];
Tz = [0, 0.2850, 0.0030, 0.1105, 0.3650, 0.0540];
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
syms th1 th2 th3 th4 th5 th6;
%ÿ���ؽ������ǰһ�ؽ�����ϵ�ı任����
T{1} = transl(Tx(1), Ty(1), Tz(1))*trotx(alpha(1))*trotz(th1);
T{2} = transl(Tx(2), Ty(2), Tz(2))*trotx(alpha(2))*trotz(th2);
T{3} = transl(Tx(3), Ty(3), Tz(3))*trotx(alpha(3))*trotz(th3);
T{4} = transl(Tx(4), Ty(4), Tz(4))*trotx(alpha(4))*trotz(th4);
T{5} = transl(Tx(5), Ty(5), Tz(5))*trotx(alpha(5))*trotz(th5);
T{6} = transl(Tx(6), Ty(6), Tz(6))*trotx(alpha(6))*trotz(th6);
%�����ؽ�����ڹ���ϵ0�ı任����
T01 = T{1};
T02 = T{1}*T{2};
T03 = T{1}*T{2}*T{3};
T04 = T{1}*T{2}*T{3}*T{4};
T05 = T{1}*T{2}*T{3}*T{4}*T{5};
T06 = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};

T_initial = subs(T06, {th1, th2, th3, th4, th5, th6},{0.5, 0.5, 1-pi/2, 1, 1, 1});
T_initial = eval(T_initial)
T_initial2 = subs(T06, {th1, th2, th3, th4, th5, th6},{(1.0e+03)*0.2398,(1.0e+03)*-0.0280,(1.0e+03) *0.0656,(1.0e+03)  *1.8108,(1.0e+03) *-0.0354,(1.0e+03) *-1.3859});
T_initial2 = eval(T_initial2)

