function [outputArg1,outputArg2] = dRyaw_dyaw(inputArg1,inputArg2)
R_yaw = [-sin(yaw),-cos(yaw),0;
         cos(yaw),-sin(yaw), 0;
         0,0,0];
end

