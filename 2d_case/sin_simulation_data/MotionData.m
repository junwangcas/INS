classdef MotionData < handle   
    properties
        x = -999;
        y = -999;
        theta = -999; 
        vx = -999;
        vy = -999;
        ax = -999;
        ay = -999;
        yawrate = -999;
        t_time = -999;
    end
    
    methods
        function obj = MotionData(t_time)
            %MOTIONDATA 构造此类的实例
            %   此处显示详细说明
            obj.t_time = t_time;
        end
        
        function outputArg = setT(obj,x, y)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj.x = x;
            obj.y = y;
        end
        
        function temp = setPose(obj, theta)
            obj.theta = theta;
        end
        
        function out = setVelocity(obj, xprim, yprim)
            obj.vx = xprim;
            obj.vy = yprim;
        end
        
        function out = setAcc(obj, xprimprim, yprimprim)
            obj.ax = xprimprim;
            obj.ay = yprimprim;
        end
        
        function out = setGyro(obj, thetaprim)
            obj.yawrate = thetaprim;
        end
    end
end

