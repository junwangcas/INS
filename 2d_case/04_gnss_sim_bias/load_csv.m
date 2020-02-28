function [data] = load_csv(csv_dir, outputdata, if_use_noise)
ref_gps_file = csv_dir + "ref_gps.csv";
ref_gps = csvread(ref_gps_file, 1);
if if_use_noise
    ref_imuacc_file = csv_dir + "accel-0.csv";
    ref_imugyro_file = csv_dir + "gyro-0.csv";
else
    ref_imuacc_file = csv_dir + "ref_accel.csv";
    ref_imugyro_file = csv_dir + "ref_gyro.csv";
end


ref_imuacc = csvread(ref_imuacc_file,1);
ref_imugyro = csvread(ref_imugyro_file,1);
ref_imugyro = ref_imugyro/180.0*pi;

time_file = csv_dir + "time.csv";
times = csvread(time_file,1);


%data;
data.IMU.t = times';
data.IMU.acc = ref_imuacc';
data.IMU.gyro = ref_imugyro';

hz_gps = 10;
data.GPS.t = zeros(length(ref_gps),1);
time_gps = times(1);
for i = 1:length(ref_gps)
    data.GPS.t(i) = time_gps;
    time_gps = time_gps + 1.0/hz_gps;
end
data.GPS.pos = ref_gps(:, 1:3)';

% save;
save(outputdata, 'data');
end

