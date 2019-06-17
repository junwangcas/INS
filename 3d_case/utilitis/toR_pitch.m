function [R_pitch] = toR_pitch(pitch)
%TOR_ROLL 此处显示有关此函数的摘要
%   此处显示详细说明
R_pitch = [cos(pitch),0,sin(pitch);
           0, 1, 0;
          -sin(pitch),0,cos(pitch)];
end

