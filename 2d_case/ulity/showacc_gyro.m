load('../data/GNSSaidedINS_data2d.mat');
x = 1:size(data2d.IMU.acc,2);
x = data2d.IMU.t;
plot(x, data2d.IMU.acc(1,:));
hold on; plot(x, data2d.IMU.acc(2,:));
legend('x','y');


%% gyro
figure(2);
plot(x, data2d.IMU.gyro(1,:));
legend('x','y');

