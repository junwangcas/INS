function [ output_args ] = writing_edges(navigation_data)
f_handle = fopen(navigation_data.file_edge,'w+');
id_edge = 0;
id_edge = write_gps_edge(f_handle,navigation_data,id_edge);
id_edge = write_speed_edge(f_handle,navigation_data,id_edge);
id_edge = write_acc_edge(f_handle,navigation_data,id_edge);
id_edge = write_gyro_edge(f_handle,navigation_data,id_edge);
id_edge = write_motionmodel_edge(f_handle,navigation_data,id_edge);
end

function [id_edge] = write_gps_edge(f_handle,navigation_data,id_edge)
% loop over the gps measurements and find the write node id;
imu_time_base = 1;
for i = 1:length(navigation_data.raw_data.GNSS.t)
    t_gps = navigation_data.raw_data.GNSS.t(i);
    gps_val = navigation_data.raw_data.GNSS.pos_ned(:,i)';
    for j = imu_time_base:length(navigation_data.raw_data.IMU.t)
        t_imu = navigation_data.raw_data.IMU.t(j);
        if abs(t_imu - t_gps) < 0.0001
            imu_time_base = j;
            % j: this is the pose id;
            id_R = 6 + (j-1)*3 +1;
            id_T = 6 + (j-1)*3 +2;
            id_V = 6 + (j-1)*3 +3;
            id_edge = id_edge +1;
            fprintf(f_handle,'%d %d %d %f %f %f\n',id_edge,1,id_T,gps_val);
            break;
        end
    end
end
end

function [id_edge] = write_speed_edge(f_handle,navigation_data,id_edge)
imu_time_base = 1;
for i = 1:length(navigation_data.raw_data.SPEEDOMETER.t)
    t_speed = navigation_data.raw_data.SPEEDOMETER.t(i);
    speed_val = navigation_data.raw_data.SPEEDOMETER.speed(i);
    for j = imu_time_base:length(navigation_data.raw_data.IMU.t)
        t_imu = navigation_data.raw_data.IMU.t(j);
        if abs(t_imu - t_speed) < 0.0001
            imu_time_base = j;
            % j: this is the pose id;
            id_R = 6 + (j-1)*3 +1;
            id_T = 6 + (j-1)*3 +2;
            id_V = 6 + (j-1)*3 +3;
            id_edge = id_edge +1;
            fprintf(f_handle,'%d %d %d %d %f\n',id_edge,2,id_R,id_V,speed_val);
            break;
        end
    end
end
end

function [id_edge] = write_acc_edge(f_handle,navigation_data,id_edge)
for j = 1:length(navigation_data.raw_data.IMU.t)-1
    t_imu = navigation_data.raw_data.IMU.t(j);
    acc_val = navigation_data.raw_data.IMU.acc(:,j)';
    % j: this is the pose id;
    id_R = 6 + (j-1)*3 +1;
    id_T = 6 + (j-1)*3 +2;
    id_V = 6 + (j-1)*3 +3;
    id_T2 = id_T + 3;
    id_edge = id_edge +1;
    fprintf(f_handle,'%d %d %d %d %d %d %f %f %f\n',id_edge,3,id_R,id_T,id_V,id_T2,acc_val);
end
end

function [id_edge] = write_gyro_edge(f_handle,navigation_data,id_edge)
for j = 1:length(navigation_data.raw_data.IMU.t)-1
    t_imu = navigation_data.raw_data.IMU.t(j);
    gyro_val = navigation_data.raw_data.IMU.gyro(:,j)';
    % j: this is the pose id;
    id_R = 6 + (j-1)*3 +1;
    id_T = 6 + (j-1)*3 +2;
    id_V = 6 + (j-1)*3 +3;
    id_R2 = id_R + 3;
    id_edge = id_edge +1;
    fprintf(f_handle,'%d %d %d %d %f %f %f\n',id_edge,4,id_R,id_R2,gyro_val);
end
end

function id_edge = write_motionmodel_edge(f_handle,navigation_data,id_edge)
for j = 1:length(navigation_data.raw_data.IMU.t)-1
    t_imu = navigation_data.raw_data.IMU.t(j);
    % j: this is the pose id;
    id_R = 6 + (j-1)*3 +1;
    id_T = 6 + (j-1)*3 +2;
    id_V = 6 + (j-1)*3 +3;
    id_T2 = id_T + 3;
    id_edge = id_edge +1;
    fprintf(f_handle,'%d %d %d %d %d\n',id_edge,5,id_T,id_V,id_T2);
end
end 