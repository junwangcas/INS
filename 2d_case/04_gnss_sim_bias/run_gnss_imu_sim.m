% param
%input_bag = '/home/m0/Documents/catkin_ws/testing_zeronoise.bag';
csv_dir1 = '/media/m0/Data/code/gnss-ins-sim/demo_saved_data/';
csv_dir2 = "2020-02-27-14-25-13/";
if_use_latest = true;
if if_use_latest
    lsdir = dir(csv_dir1);
    csv_dir2 = lsdir(length(lsdir)).name + "/";
end
csv_dir = csv_dir1 + csv_dir2;
outputdata = './data/datamat';
if_need_generate_data = true;
if_use_noise = true;


if if_need_generate_data
    load_csv(csv_dir, outputdata, if_use_noise);
end
plot_turtlebotdata(outputdata);


% run ekf;
if_fuse_gps = true;
exe_kf(outputdata, if_fuse_gps);


