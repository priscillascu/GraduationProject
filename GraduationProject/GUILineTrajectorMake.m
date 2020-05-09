function [sum_v, v, lx, ly, lz, vx, vy, vz] = GUILineTrajectorMake( Tx, p0, pt )
%本函数用于在GUI程序中进行调用，进行直线轨迹规划
v0 = 0;
v1 = sqrt((p0(1)-pt(1))^2+(p0(2)-pt(2))^2+(p0(3)-pt(3))^2)/(16*Tx/7);  %设置第一个加速度拐点速度v1

dt = Tx/7;
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
T = 0 : 0.1 : Tx;

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
lx = p0(1)+sum_v*(pt(1)-p0(1))/sqrt((pt(1)-p0(1))^2 + (pt(2)-p0(2))^2);
ly = p0(2)+sum_v*(pt(2)-p0(2))/sqrt((pt(1)-p0(1))^2 + (pt(2)-p0(2))^2);
lz = p0(3)+sum_v*(pt(3)-p0(3))/sqrt((pt(1)-p0(1))^2 + (pt(2)-p0(2))^2 + (pt(3)-p0(3))^2);

vx = v*(pt(1)-p0(1))/sqrt((pt(1)-p0(1))^2 + (pt(2)-p0(2))^2);
vy = v*(pt(2)-p0(2))/sqrt((pt(1)-p0(1))^2 + (pt(2)-p0(2))^2);
vz = v*(pt(3)-p0(3))/sqrt((pt(1)-p0(1))^2 + (pt(2)-p0(2))^2 + (pt(3)-p0(3))^2);
end

