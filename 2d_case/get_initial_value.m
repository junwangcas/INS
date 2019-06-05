function [ X, P] = get_initial_value(poses)
pose1 = poses(:,1);
pose2 = poses(:,2);
theta = atan2(pose2(2)-pose1(2), pose2(1)-pose1(1));
v_x = pose2(1) - pose1(1);
v_y = pose2(2) - pose1(2);
s = pose1;
X = [theta;v_x;v_y;s;0];

P = eye(6,6);
P(1,1) = 0.1;
P(2:3,2:3) = eye(2,2);
P(4:5,4:5) = eye(2,2);
end

