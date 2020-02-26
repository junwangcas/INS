clear all;
%load('../data/GPSaidedINS_data2d.mat');
%load('../data/turtlebot_data2d.mat');
load('../data/left_trans3.mat');
%load('sin_simulation_data/sinmotiondata2d.mat');
%data2d = select_data(data2d);
if_fuse_gps = false;

% initial value;
X = [0;0;0;0;0];
[X, P] = get_initial_value(data2d.GPS.pos_EN, 2, 1);

% for sin simulation
% X(1) = 1.107148717794090;
% X(2) = 1;
% X(3) = 2;

it_imu = 0;
it_gps = 0;
X_list = X;
P_list{1} = P;
delta_t_list = [];
% plot gps;
for id_t_imu = 1:length(data2d.IMU.t) -1
    disp(id_t_imu);
    t_imu = data2d.IMU.t(id_t_imu);
    %% predict;
    it_imu = it_imu + 1;
    delta_t = data2d.IMU.t(id_t_imu + 1) - data2d.IMU.t(id_t_imu);
    delta_t_list = cat(1, delta_t_list,delta_t);
    aL = data2d.IMU.acc(:, it_imu);
    yaw_rate = data2d.IMU.gyro(1, it_imu);
    [X_bar, F] = get_state_transition_F(delta_t,yaw_rate,aL,X(1),X(2:3),X(4:5));
    Q  = get_Q();
    P_bar = F*P*F' + Q;
    
    %% update
    % check if the t equals the gps 
    if (if_fuse_gps && it_gps + 1 <= length(data2d.GPS.t)) && (t_imu == data2d.GPS.t(it_gps + 1))
        it_gps
        it_gps = it_gps + 1;
        H = zeros(2,5);
        H(1:2,4:5) = eye(2,2);
        R = 0.1*eye(2,2);
        z = data2d.GPS.pos_EN(:,it_gps);
        y = z - H*X_bar;
        K = P_bar*H'*inv(H*P_bar*H' + R);
        X = X_bar + K*y;
        X(1) = normalize_theta(X(1));
        P = (eye(5,5) - K*H)*P_bar;
        disp('get a gps');
        %pause(0.01);
        %if (mod(it_gps, 3) == 0)
%             clf;
%             scatter(X_list(4,:), X_list(5,:));
%             hold on; scatter(data2d.GPS.pos_EN(1,:),data2d.GPS.pos_EN(2,:));
%             xlabel('x-E');ylabel('y-north');
%             waitforbuttonpress;
        %end
    else
        X = X_bar;
        P = P_bar;
    end
    X_list = cat(2, X_list, X);
    P_list = cat(1, P_list, P);
end

% check;
%plot(delta_t_list);
plot(X_list(1,:));


figure;
scatter(X_list(4,:), X_list(5,:));
hold on; scatter(data2d.GPS.pos_EN(1,:),data2d.GPS.pos_EN(2,:));
xlabel('x-E');ylabel('y-north');
title('position');

% figure; 速度
figure;
x = 1:length(X_list(2,:));
plot(x,X_list(2,:));
hold on; plot(x,X_list(3,:));
xlabel('time');ylabel('x, y speed');
legend('x speed', 'y speed');
title('speed');

% yaw;
figure;
x = 1:length(X_list(2,:));
plot(x,X_list(1,:));
title('yaw');

% acc-local
figure;
subplot(2,1,1);
plot(data2d.IMU.acc(1,:));
subplot(2,1,2);
plot(data2d.IMU.acc(2,:));
title('acc xy local');

% acc-global
figure;
subplot(2,1,1);
thetas = X_list(1,:);
acc_global = zeros(2,length(data2d.IMU.acc));
for i = 1:length(data2d.IMU.acc)
    acc_global(:,i) = to_R2d(thetas(i))*data2d.IMU.acc(:,i);
end
subplot(2,1,1);
plot(acc_global(1,:));
subplot(2,1,2);
plot(acc_global(2,:));
title('acc xy global');


