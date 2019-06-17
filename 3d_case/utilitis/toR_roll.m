function [R_roll] = toR_roll(roll)
%TOR_ROLL 此处显示有关此函数的摘要
%   此处显示详细说明
R_roll = [1,0,0;
          0,cos(roll),-sin(roll);
          0,sin(roll),cos(roll)];
end

