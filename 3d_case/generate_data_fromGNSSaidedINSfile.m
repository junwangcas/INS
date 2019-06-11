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

% Gyro roll
figure();
plot(in_data.IMU.t, in_data.IMU.gyro(1,:));
xlabel('time');ylabel('value');
title('Gyro roll');

% Gyro pitch
figure();
plot(in_data.IMU.t, in_data.IMU.gyro(2,:));
hold on; plot(in_data.GNSS.t,in_data.GNSS.pos_ned(3,:)/80.0);
xlabel('time');ylabel('value');
title('Gyro pitch, GPS_D/80');
legend('gyro pitch', 'gps d');

% acc x;
figure();
plot(in_data.IMU.t, in_data.IMU.acc(1,:));
idx = 1:length(in_data.GNSS.t)-1;
speed_n = in_data.GNSS.pos_ned(1,idx+1) - in_data.GNSS.pos_ned(1,idx);
speed_e = in_data.GNSS.pos_ned(2,idx+1) - in_data.GNSS.pos_ned(2,idx);
speed_ne = sqrt(speed_n.^2 + speed_e.^2);
hold on; plot(in_data.GNSS.t(idx), speed_ne); 
title('acc x');
legend('imu acc', 'gps speed');

% acc y;
figure();
plot(in_data.IMU.t, in_data.IMU.acc(1,:));
hold on; plot(in_data.IMU.t, in_data.IMU.acc(2,:));
hold on; plot(in_data.GNSS.t(idx), speed_ne); 
legend('imu accx','imu accy', 'gps speed');
title('acc y');

% acc z;
figure();
plot(in_data.IMU.t, in_data.IMU.acc(3,:)+9.8);
hold on; plot(in_data.GNSS.t,in_data.GNSS.pos_ned(3,:)/80.0);
title('acc z, gps z');
legend('acc z', 'gps d');



