function [ th,xc,yc,zc ] = GUITrajControl( pos0, th0, p0, pt, lx, ly, lz, vx, vy, vz, T, ctime)
%基于固定时间的轨迹规划
dth = [0; 0; 0; 0; 0; 0];
x1 = [0; 0; 0; 0; 0; 0];

intx = [0; 0; 0; 0; 0; 0];
e = [0; 0; 0; 0; 0; 0];
de = [0; 0; 0; 0; 0; 0];
intxd = [0; 0; 0; 0; 0; 0];
x_before = [0; 0; 0; 0; 0; 0];
x = [0.9331; 0.0404; 1.0982; 0; 0; 0];  %初始位置
xc = x(1);
yc = x(2);
zc = x(3);

th = th0; %初始角度

%0时刻的误差值e0
e0 = [0;0;0; 0; 0; 0];
de0 = [0; 0; 0; 0; 0; 0];
dde0 = [0; 0; 0; 0; 0; 0];
c1 = 1.3*[1; 1; 1; 1; 1; 1];
C = [c1];
lamda = 1;
% 
%针对初始角度不为0的时候
%0时刻的误差值e0
% e0 = [pos0(1)-p0(1); pos0(2)-p0(2); pos0(3)-p0(3); 0; 0; 0];
% de0 = [0; 0; 0; 0; 0; 0];
% dde0 = [0; 0; 0; 0; 0; 0];
% c1 = 1.1*[0.8; 0.8; 0.8; 1; 1; 1];
% C = [c1];
% K = 1.7;


%P算子中的系数a
ctime = ctime;
a00 = -3;
a10 = 2;
a01 = -2;
a11 = 1;

for i = 1 : T*10
    %位置和速度给定 位置积分给定、位置积分给定
    xd = [lx(i); ly(i); lz(i); 0; 0; 0];
    dxd = [vx(i); vy(i); vz(i); 0; 0; 0];
    intxd = intxd + xd;
    
    %解出当前位置和位置积分
    Tc = myFKSolver(th(:, i));
    xc(i+1) = Tc(1, 4);
    yc(i+1) = Tc(2, 4);
    zc(i+1) = Tc(3, 4);
    x_before = x;
    x = [xc(i+1); yc(i+1); zc(i+1); 0; 0; 0];
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


end

