function [outputArg1,outputArg2] = exe_kf(input_bag)
output_bag = input_bag + "trans";
transtodata2d(input_bag, output_bag);
load(output_bag);
if_fuse_gps = false;

% initial value;
X = [0;0;0;0;0;0;0;0];
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
figure;
subplot(3,3,1);
plot(X_list(1,:));
title('yaw estimation');

subplot(3,3,2);
scatter(X_list(4,:), X_list(5,:));
hold on; scatter(data2d.GPS.pos_EN(1,:),data2d.GPS.pos_EN(2,:),'*');
xlabel('x-E');ylabel('y-north');
title('estimate/GT position');

% figure; 速度
subplot(3,3,3);
x = 1:length(X_list(2,:));
plot(x,X_list(2,:));
hold on; plot(x,X_list(3,:));
xlabel('time');ylabel('x, y speed');
legend('x speed', 'y speed');
title('estimate speed');

% acc-local
subplot(3,3,4);
plot(data2d.IMU.acc(1,:));
title('acc x local');
subplot(3,3,5);
plot(data2d.IMU.acc(2,:));
title('acc y local');

% acc-global
thetas = X_list(1,:);
acc_global = zeros(2,length(data2d.IMU.acc));
for i = 1:length(data2d.IMU.acc)
    acc_global(:,i) = to_R2d(thetas(i))*data2d.IMU.acc(:,i);
end
subplot(3,3,6);
plot(acc_global(1,:));
title('acc x global');
subplot(3,3,7);
plot(acc_global(2,:));
title('acc y global');
end

function transtodata2d(input_bag, output_bag)
%load('/home/nvidia/code/INS/data/GPSaidedINS_data.mat');
load(input_bag);

% GPS;
data2d.GPS.t = data.GPS.t;
data2d.GPS.pos_EN = data.GPS.pos(1:2,:);
% imu
data2d.IMU.t = data.IMU.t;
data2d.IMU.acc = data.IMU.acc(1:2,:);
data2d.IMU.gyro = data.IMU.gyro(3,:);
save(output_bag, 'data2d');
end
