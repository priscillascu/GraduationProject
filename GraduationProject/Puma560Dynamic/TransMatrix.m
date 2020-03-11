clear all
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
a = [0, 0, 0.5, 0, 0, 0];
d = [0, 0.125, 0, 0.25, 0, 0];
th = [0, 0, -pi/2, 0, 0, 0];
T0 = 1;
for i = 1:6
    T{i} = trotx(alpha(i))*transl(0, 0, d(i))*transl(a(i), 0, 0)*trotz(th(i));
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