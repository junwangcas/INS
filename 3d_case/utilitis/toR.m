function [R] = toR(flag, roll, pitch, yaw)
if flag == 1
    yaw = roll(3);
    pitch = roll(2);
    roll = roll(1);
else flag == 0
    % do nothing;
end
R_roll = toR_roll(roll);
R_pitch = toR_pitch(pitch);
R_yaw = toR_yaw(yaw);
R = R_yaw*R_pitch*R_pitch;
end

