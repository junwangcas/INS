clear
load('/home/nvidia/code/INS/data/GNSSaidedINS_data.mat');
% gnss;
data2d.GNSS.t = in_data.GNSS.t;
data2d.GNSS.pos_EN = in_data.GNSS.pos_ned(2:-1:1,:);
% imu
data2d.IMU.t = in_data.IMU.t;
data2d.IMU.acc = in_data.IMU.acc(1:2,:);
data2d.IMU.gyro = in_data.IMU.gyro(3,:);
save('../data/GNSSaidedINS_data2d', 'data2d');

x = 1:size(in_data.IMU.gyro,2);
plot(x, in_data.IMU.gyro(1,:));
hold on; plot(x, in_data.IMU.gyro(2,:));
hold on; plot(x, in_data.IMU.gyro(3,:));
legend('1','2','3');
