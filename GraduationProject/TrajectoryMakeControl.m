clear all;

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

%%
%ʹ��ָ�������ɵĻ�ģ����
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
%     %λ�ú��ٶȸ���
%     xd = [lx(i); ly(i); lz(i); 0; 0; 0];
%     dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
%     %�����ǰλ��
%     Tc = myFKSolver(th(:, i));
%     xc(i) = Tc(1, 4);
%     yc(i) = Tc(2, 4);
%     zc(i) = Tc(3, 4);
%     x = [xc(i); yc(i); zc(i); 0; 0; 0];
%     %��⵱ǰ�Ƕ��µ��ſɱȾ���
%     Jac = JacobianCross(th(:, i));
%     %���
%     e(:, i) = x - xd;
%     s = c*e(i);
%     
%     dth(:, i) = inv(Jac)*(dxd - ita*tanh(s/0.1) - k*abs(s).^alpha*tanh(s/0.1));
%     th(:, i + 1) = th(:, i) + dth(:, i)*0.1;
% end

%%
%���ڷ������ն˻�ģ�Ĺ켣�滮
dth = [0; 0; 0; 0; 0; 0];


x1 = [0; 0; 0; 0; 0; 0];
xc = [0];
yc = [0];
zc = [0];
intx = [0; 0; 0; 0; 0; 0];
e = [0; 0; 0; 0; 0; 0];
de = [0; 0; 0; 0; 0; 0];
intxd = [0; 0; 0; 0; 0; 0];

q = 3;
p = 5;

%��Գ�ʼ�Ƕ�Ϊ0��ʱ��
th = [0; 0; 0; 0; 0; 0]; %��ʼ�Ƕ�
beta = 30;
xite = 2;

%��Գ�ʼ�ǶȲ�Ϊ0��ʱ��
% beta = 8.5;
% xite = 0.001;
% th = [1; 1; 1; 1; 1; 1];  %��ʼ�Ƕ�

for i = 1 : length(T)
    %λ�ú��ٶȸ��� λ�û��ָ�����λ�û��ָ���
    xd = [lx(i); ly(i); lz(i); 0; 0; 0];
    dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
    intxd = intxd + xd;
    
    %�����ǰλ�ú�λ�û���
    Tc = myFKSolver(th(:, i));
    xc(i) = Tc(1, 4);
    yc(i) = Tc(2, 4);
    zc(i) = Tc(3, 4);
    x = [xc(i); yc(i); zc(i); 0; 0; 0];
    intx = intx + x;
    
    e(:, i) = intx - intxd;
    de(:, i) = x - xd;
    
    Jac = JacobianCross(th(:, i));
    T1 = abs(de(:, i)).^(p/q).*tanh(de(:, i)/0.1);
    T2 = abs(de(:, i)).^(2-p/q).*tanh(de(:, i)/0.1);   %��ֱ����x2^(2-p/q)�����ּ��������Ϊ2-p/q����Ϊ������x2����С��0
    s = e(:, i) + 1/beta*T1;
    
    dth(:, i) = -inv(Jac)*((beta*q/p)*T2 + xite*tanh(s/0.1));

    th(:, i + 1) = th(:, i) + dth(:, i)*0.1;
end

%%
%ʹ�õ��������ؽڽǶ�
% lamda = 1;
% dth = [0; 0; 0; 0; 0; 0];
% th = [0; 0; 0; 0; 0; 0];
% for i = 1: length(T)
%     xd = [lx(i); ly(i); lz(i); 0; 0; 0];
%     dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
%     for j = 1: 1: 1000
%         Jac = JacobianCross(th);
%         Tc = myFKSolver(th);
%         angle = tr2rpy(Tc);
%         xc(i) = Tc(1, 4);
%         yc(i) = Tc(2, 4);
%         zc(i) = Tc(3, 4);
%         dx = xd - [xc(i); yc(i); zc(i); 0; 0; 0]
%         %dth(: , i+1) = Jac'*inv(Jac*Jac' + lamda^2 .* diag([1 1 1 1 1 1], 0))*dx;
%         dth(:, i+1) = inv(Jac)*dx;
%         th = th + dth(: , i+1)*0.1;
%         if norm(dx) < 0.00001
%             disp('������')
%             break
%         else
%             disp('�����')
%         end
%     end
% end

%%
%��ͼ
figure
plot(T, lx);
hold on
plot(T, xc);
legend('����', 'ʵ��');
figure
plot(T, ly);
hold on
plot(T, yc);
figure
plot(T, lz);
hold on
plot(T, zc);

figure
subplot(2, 3, 1);
plot(T, dth(1, :));
subplot(2, 3, 2);
plot(T, dth(2, :));
subplot(2, 3, 3);
plot(T, dth(3, :));
subplot(2, 3, 4);
plot(T, dth(4, :));
subplot(2, 3, 5);
plot(T, dth(5, :));
subplot(2, 3, 6);
plot(T, dth(6, :));

figure
subplot(1, 3, 1);
plot(T, e(1, :));
subplot(1, 3, 2);
plot(T, e(2, :));
subplot(1, 3, 3);
plot(T, e(3, :));

figure
subplot(1, 3, 1);
plot(T, de(1, :));
subplot(1, 3, 2);
plot(T, de(2, :));
subplot(1, 3, 3);
plot(T, de(3, :));