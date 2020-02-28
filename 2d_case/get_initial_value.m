function [ X, P] = get_initial_value(poses, end_idx, start_idx)
pose1 = poses(:,start_idx);
pose2 = poses(:,end_idx);
if abs(pose1(1)-pose2(1)) < 1e-4
    theta = 0;
else
    theta = atan2(pose2(2)-pose1(2), pose2(1)-pose1(1));
end
v_x = pose2(1) - pose1(1);
v_y = pose2(2) - pose1(2);
X = [pose1; v_x; v_y; theta;0;0;0];

P = eye(8,8);
end

