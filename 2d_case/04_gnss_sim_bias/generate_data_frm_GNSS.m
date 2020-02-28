function [] = generate_data_frm_turtlebot(input_bag, outputdata)
bag = rosbag(input_bag);

%
bag_imu = select(bag,'Topic','/imu');
bag_modelstate = select(bag,'Topic','/gazebo/model_states');
%% imu的频率200hz; model的频率1000hz;
t_imu  = bag_imu.MessageList{:,1};
t_model = bag_modelstate.MessageList{:,1};
%% imu中的一个数值，对应一个model中的数值；
[c, i_imu, i_model] = intersect(t_imu,t_model);

%% imu选择原始频率，pose选择10hz;
interval_gps = 1/10.0; 
data.IMU.t = [];
data.IMU.acc =[];
data.IMU.gyro = [];
data.GPS.t = [];
data.GPS.pos = [];
data.GPS.altitude = [];
t_prev_gps = t_model(i_model(1));

for i = 1:length(i_imu)
    if (mod(i,1000) == 0)
        i
    end
    data.IMU.t = cat(2, data.IMU.t, bag_imu.MessageList{i_imu(i),1});
    imu_msgs = readMessages(bag_imu, i_imu(i));
    acc = [imu_msgs{1}.LinearAcceleration.X; imu_msgs{1}.LinearAcceleration.Y;imu_msgs{1}.LinearAcceleration.Z];
    gyro = [imu_msgs{1}.AngularVelocity.X;imu_msgs{1}.AngularVelocity.Y;imu_msgs{1}.AngularVelocity.Z];
    data.IMU.acc = cat(2, data.IMU.acc, acc);
    data.IMU.gyro = cat(2, data.IMU.gyro, gyro);
    
    gps_msgs = readMessages(bag_modelstate, i_model(i));
    gps_pose = gps_msgs{1}.Pose(2);
    gps_position = [gps_pose.Position.X; gps_pose.Position.Y; gps_pose.Position.Z];
    gps_altitude = [gps_pose.Orientation.X; gps_pose.Orientation.Y; gps_pose.Orientation.Z; gps_pose.Orientation.W]; 

    if (i == 1)
        t_prev_gps = t_model(i_model(i));
        data.GPS.t = cat(2, data.GPS.t, t_model(i_model(i)));
        data.GPS.pos = cat(2, data.GPS.pos, gps_position);
        data.GPS.altitude = cat(2, data.GPS.altitude, gps_altitude);
    elseif (t_model(i_model(i)) - t_prev_gps >= interval_gps) 
        t_prev_gps = t_model(i_model(i));
        data.GPS.t = cat(2, data.GPS.t, t_model(i_model(i)));
        data.GPS.pos = cat(2, data.GPS.pos, gps_position);
        data.GPS.altitude = cat(2, data.GPS.altitude, gps_altitude);
    end
end

save(outputdata, 'data');
end

