clear;
load('../data/turtlebot1');
% draw gps； 全局坐标系
plot(data.GPS.pos(1,:),data.GPS.pos(2,:));
title('gps data');
xlabel('x-E');ylabel('y-N');

% draw imu acc x, yaw; 
figure;
plot(data.IMU.t,data.IMU.acc(1,:));
hold on; plot(data.IMU.t,data.IMU.gyro(3,:));
hold on;plot(data.GPS.t,data.GPS.pos(1,:));
title('imu acc x, gyro yaw');
xlabel('t');ylabel('value');
legend('imu acc x', 'gyro yaw', 'gps x');
ylim([-5,5]);


% draw imu acc y, acc z; 
figure;
plot(data.IMU.t,data.IMU.acc(2,:));
hold on;plot(data.IMU.t,data.IMU.acc(3,:));
title('imu acc y z');
xlabel('t');ylabel('value');
legend('imu acc y','imu acc z');
ylim([-5,5]);

% draw gyro roll pitch;
figure;
plot(data.IMU.t, data.IMU.gyro(1,:));
hold on; plot(data.IMU.t, data.IMU.gyro(2,:));
title('imu gyro roll pitch');
legend('gyro roll','gyro pitch');
ylim([-0.05,0.05]);


