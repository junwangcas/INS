clear
load('../data/GNSSaidedINS_data.mat');
% gnss;
data3d.GNSS.t = in_data.GNSS.t;
data3d.GNSS.pos_NED = in_data.GNSS.pos_ned;

figure();
plot(data3d.GNSS.pos_NED(2,:),data3d.GNSS.pos_NED(1,:));
xlabel('y-E');ylabel('x-north');
title('GPS position');

figure();
plot(in_data.GNSS.HDOP);
hold on;plot(in_data.GNSS.VDOP);
legend('h','v');
title('GPS speed');


% Gyro yaw;
figure();
plot(in_data.IMU.t,in_data.IMU.gyro(3,:));
idx = 1:length(in_data.GNSS.t)-1;
speed_n = in_data.GNSS.pos_ned(1,idx+1) - in_data.GNSS.pos_ned(1,idx);
speed_e = in_data.GNSS.pos_ned(2,idx+1) - in_data.GNSS.pos_ned(2,idx);
speed_yaw = atan2(speed_e,speed_n);
hold on;plot(in_data.GNSS.t(idx),speed_yaw);
legend('IMU gyro yaw rate', 'gps yaw');
xlabel('time');ylabel('value');



