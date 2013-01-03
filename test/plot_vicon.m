#!/usr/bin/octave
DATA = load('1LoopDown/vicon_1loopDown.txt');
first = 1;
step = 1;
last = 1550;
T = DATA(first:step:end,1);
XYZ = DATA(first:step:end,2:4);
RPY = DATA(first:step:end,5:7);
W = DATA(first:step:end,8:10);

T = T - T(1);

figure;
plot(T,[gradient(XYZ(:,1),T) gradient(XYZ(:,2),T) gradient(XYZ(:,3),T)]);
legend('x','y','z');
title('vicon linear velocity');

figure;
plot(T,180/pi*W);
legend('x','y','z');
title('vicon angular velocity');

figure;
plot(T,180/pi*RPY);
legend('r','p','y');
title('vicon orientation (Euler angles)');

figure;
plot3c(XYZ(:,1), XYZ(:,2), XYZ(:,3), T);
axis([-3 3 -5 -1.5 0.5 2], 'equal');
cb_h = colorbar();
ylabel(cb_h, 'flight time (s)');
xlabel('x');
ylabel('y');
zlabel('z');
title('vicon trajectory');

pause;
