#!/usr/bin/octave
DATA = load('simple_test_filter.txt');
first = 1;
step = 1;
last = 100;
T  = DATA(first:step:end,1);
W  = DATA(first:step:end,2:4);
D  = DATA(first:step:end,5:7);
R1 = DATA(first:step:end,8:10);
R2 = DATA(first:step:end,11:13);
R3 = DATA(first:step:end,14:16);

T = T - T(1);

figure
hold on;
P = (1:rows(T))' * [1 0 0];
quiver3(P(:,1), P(:,2), P(:,3), R1(:,1),R1(:,2),R1(:,3),0,'b');
quiver3(P(:,1), P(:,2), P(:,3), R2(:,1),R2(:,2),R2(:,3),0,'g');
quiver3(P(:,1), P(:,2), P(:,3), R3(:,1),R3(:,2),R3(:,3),0,'r');
%quiver(P(:,1), P(:,2), R1(:,1), R1(:,2), 0, 'b');
%quiver(P(:,1), P(:,2), R2(:,1), R2(:,2), 0, 'g');
xl = get (gca, "xlim");
yl = get (gca, "ylim");
zl = get (gca, "zlim");
%span = max([diff(xl), diff(yl)]);
span = max([diff(xl), diff(yl), diff(zl)]);
xlim (mean (xl) + span*[-0.5, 0.5])
ylim (mean (yl) + span*[-0.5, 0.5])
zlim (mean (zl) + span*[-0.5, 0.5]);
axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
title('Orientation');

figure
plot(T,W);
legend({'x','y','z'});
title('Angular velocity');

figure
plot(T,D);
legend({'x','y','z'});
ylim([min(D(:)),max(D(:))]+0.1*[-1 1]*range(D(:)))
title('Gyroscope drift');

pause;
