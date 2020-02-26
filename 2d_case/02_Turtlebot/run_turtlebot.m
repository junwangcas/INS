% param
input_bag = '/home/m0/Documents/catkin_ws/testing_zeronoise.bag';
outputdata = './data/testing_zeronoise';
if_need_generate_data = false;
addpath('../');


if if_need_generate_data
    generate_data_frm_turtlebot(input_bag, outputdata);
end
plot_turtlebotdata(outputdata);


% run ekf;
exe_kf(outputdata);