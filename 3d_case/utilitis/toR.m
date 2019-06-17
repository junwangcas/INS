function [R] = toR(roll, pitch, yaw)
R_roll = toR_roll(roll);
R_pitch = toR_pitch(pitch);
R_yaw = toR_yaw(yaw);
R = R_yaw*R_pitch*R_pitch;
end

