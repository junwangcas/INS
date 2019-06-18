function [X, P] = get_initial_value(poses)
pose1 = poses(:,1);
pose2 = poses(:,2);
roll = 0;
pitch = 0;
if abs(pose1(1)-pose2(1)) < 1e-4
    yaw = 0;
else
    yaw = atan2(pose2(2)-pose1(2), pose2(1)-pose1(1));
end

v_x = (pose2(1) - pose1(1))/0.1;
v_y = (pose2(2) - pose1(2))/0.1;
v_z = 0;
s = pose1;
X = [roll; pitch; yaw; v_x; v_y; v_z; s];

P = eye(9,9);
P(1:3,1:3) = 0.1;
P(4:6,4:6) = eye(3,3);
P(7:9,7:9) = eye(3,3);
end

