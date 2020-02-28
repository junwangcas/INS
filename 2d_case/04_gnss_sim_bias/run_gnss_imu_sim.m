% param
%input_bag = '/home/m0/Documents/catkin_ws/testing_zeronoise.bag';
csv_dir = '/media/m0/Data/code/gnss-ins-sim/demo_saved_data/2020-02-27-14-25-13/';
outputdata = './data/datamat';
if_need_generate_data = false;
if_use_noise = true;


if if_need_generate_data
    load_csv(csv_dir, outputdata, if_use_noise);
end
plot_turtlebotdata(outputdata);


% run ekf;
exe_kf(outputdata);