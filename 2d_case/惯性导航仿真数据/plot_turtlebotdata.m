clear;
load('./data/left3');
% draw gps； 全局坐标系
plot(data.GPS.pos(1,:),data.GPS.pos(2,:));
axis equal;
title('gps data');
xlabel('x-E');ylabel('y-N');

% draw imu acc x, y, z
figure;
subplot(3,1,1);
plot(data.IMU.t, data.IMU.acc(1,:));
title('imu acc x');
subplot(3,1,2);
plot(data.IMU.t, data.IMU.acc(2,:));
title('imu acc y');
subplot(3,1,3);
plot(data.IMU.t, data.IMU.acc(3,:));
title('imu acc z');
xlabel('t');ylabel('value');
%legend('imu acc x', 'imu acc y', 'imu acc z');
ylim([-5,5]);

% draw gyro roll/pitch/yaw
figure;
subplot(3,1,1);
plot(data.IMU.t, data.IMU.gyro(1,:));
title('gyro roll');
subplot(3,1,2);
plot(data.IMU.t, data.IMU.gyro(2,:));
title('gyro pitch');
subplot(3,1,3);
plot(data.IMU.t, data.IMU.gyro(3,:));
title('gyro yaw');
%legend('imu acc y','imu acc z');
ylim([-5,5]);
