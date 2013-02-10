#!/usr/bin/octave
DATA = load('test_sfly_filter.txt');
first = 1;
step = 1;
last = size(DATA,1);
T = DATA(first:step:last,1);
P = DATA(first:step:last,2:4);
V = DATA(first:step:last,5:7);
B = DATA(first:step:last,8:10);
W = DATA(first:step:last,11:13);
D = DATA(first:step:last,14:16);

T = T - T(1);

figure;
plot3c(P(:,1),P(:,2),P(:,3),T);
axis([-3 3 -5 -1.5 0.5 2], 'equal');
xlabel('x');
ylabel('y');
zlabel('z');
cb_h = colorbar();
ylabel(cb_h, 'flight time (s)');
title('Trajectory');
view(0,90);
print(gcf, 'trajectory_xy.png');
view(0,0);
print(gcf, 'trajectory_xz.png');

figure;
plot(T,V);
grid('on');
legend('x','y','z');
title('Linear velocity');
print(gcf, 'linear_velocity.png');

figure;
plot(T,W);
grid('on');
legend('x','y','z');
title('Angular velocity');
print(gcf, 'angular_velocity.png');

figure;
plot(T,B);
grid('on');
legend('x','y','z');
title('Accelerometer bias');
print(gcf, 'accelerometer_bias.png');

figure;
plot(T,D*180/pi);
grid('on');
legend('x','y','z');
title('Gyroscope drift');
print(gcf, 'gyroscope_drift.png');

pause;
