#!/usr/bin/octave
DATA = load('1LoopDown/imu_1loopDown.txt');
first = 1;
step = 1;
last = 1550;
T = DATA(first:step:end,1);
A = DATA(first:step:end,2:4);
W = DATA(first:step:end,5:7);
RPY = DATA(first:step:end,8:10);

T = T - T(1);

figure;
plot(T, 180/pi*W);
title('imu gyroscope output');
legend('x','y','z');

figure;
plot(T, A);
title('imu accelerometer output');
legend('x','y','z');

figure;
plot(T, 180/pi*RPY);
title('imu orientation (Euler angles)');
legend('r','p','y');

pause;

