function [ J ] = JacobianCross( th )
%本函数用于求解CRP14机械臂在th角度下的雅可比矩阵，使用向量积法
Tx = [0, 0.1949, 1.0938e-04, 0.2000, 0, -4.7162e-04];
Ty = [0, -0.0951, -0.6137, 0.2750, 0.0320, 0.0981];
Tz = [0, 0.2850, 0.0030, 0.1105, 0.3650, 0.0540];
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
%每个关节相对于前一关节坐标系的变换矩阵
T{1} = transl(Tx(1), Ty(1), Tz(1))*trotx(alpha(1))*trotz(th(1));
T{2} = transl(Tx(2), Ty(2), Tz(2))*trotx(alpha(2))*trotz(th(2));
T{3} = transl(Tx(3), Ty(3), Tz(3))*trotx(alpha(3))*trotz(th(3)-pi/2);
T{4} = transl(Tx(4), Ty(4), Tz(4))*trotx(alpha(4))*trotz(th(4));
T{5} = transl(Tx(5), Ty(5), Tz(5))*trotx(alpha(5))*trotz(th(5));
T{6} = transl(Tx(6), Ty(6), Tz(6))*trotx(alpha(6))*trotz(th(6));
%各个关节相对于惯性系0的变换矩阵
T01 = T{1};
T02 = T{1}*T{2};
T03 = T{1}*T{2}*T{3};
T04 = T{1}*T{2}*T{3}*T{4};
T05 = T{1}*T{2}*T{3}*T{4}*T{5};
T06 = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
%末端坐标系在各个关节下的齐次变换矩阵
T16 = T{2}*T{3}*T{4}*T{5}*T{6};
T26 = T{3}*T{4}*T{5}*T{6};
T36 = T{4}*T{5}*T{6};
T46 = T{5}*T{6};
T56 = T{6};
%各个关节坐标系相对于基座标系的旋转矩阵
R01 = T01(1:3, 1:3);
R02 = T02(1:3, 1:3);
R03 = T03(1:3, 1:3);
R04 = T04(1:3, 1:3);
R05 = T05(1:3, 1:3);
R06 = T06(1:3, 1:3);
%取旋转矩阵第3列，即Z轴方向分量
Z1 = R01(: , 3);
Z2 = R02(: , 3);
Z3 = R03(: , 3);
Z4 = R04(: , 3);
Z5 = R05(: , 3);
Z6 = R06(: , 3);
%末端关节坐标系相对于前面各个坐标系的位置，即齐次变换矩阵的第四列
P16 = T16(1:3, 4);
P26 = T26(1:3, 4);
P36 = T36(1:3, 4);
P46 = T46(1:3, 4);
P56 = T56(1:3, 4);
P66 = [0; 0; 0];
%使用向量积求出雅可比矩阵
J1 = [cross(Z1, R01*P16); Z1];
J2 = [cross(Z2, R02*P26); Z2];
J3 = [cross(Z3, R03*P36); Z3];
J4 = [cross(Z4, R04*P46); Z4];
J5 = [cross(Z5, R05*P56); Z5];
J6 = [cross(Z6, R06*P66); Z6];

J = [J1, J2, J3, J4, J5, J6];

end

