% param
%input_bag = '/home/m0/Documents/catkin_ws/testing_zeronoise.bag';
csv_dir = '/media/m0/Data/code/gnss-ins-sim/demo_saved_data/2020-02-26-17-53-19/';
outputdata = './data/testing_zeronoise';
if_need_generate_data = false;
addpath('../');


if if_need_generate_data
    load_csv(csv_dir, outputdata);
end
plot_turtlebotdata(outputdata);


% run ekf;
exe_kf(outputdata);