file = '/home/m0/data/turtlebot/2019-06-11-08-26-40.bag';
bag = rosbag(file);

%
bag_imu = select(bag,'Topic','/imu');
bag_modelstate = select(bag,'Topic','/gazebo/model_states');

% 时间戳对齐。不同的算法；