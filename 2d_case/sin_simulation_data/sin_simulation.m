clear;
% 定义结构体
param.a = 1;
param.b = 1;
param.c = 2;
param.d = 3;
param.m = 1;
param.n = -1;
param.t_start = 0.0; %时间起始时间，０
param.t_end = 15;  % 持续时间
param.imu_timestep = 1.0/100.0;


a = param.a;
b = param.b;
c = param.c;
d = param.d;
m = param.m;
n = param.n;

xs = [];
ys = [];
t_time = param.t_start;
motiondatas = [];
while (t_time < param.t_end)
    % position;
    x = m*(t_time + n);
    y = c * sin(a * x + b) + d;
    xs = cat(1, xs, x);
    ys = cat(1, ys, y);
    % 速度
    xprim = m;
    yprim = a*c*m*cos(a*x+b);

    %角速度
    k = a*c*cos(a*x+b);
    theta = atan(k);
    thetaprim = -1.0/(1+k*k)*a*a*c*m*sin(a*x + b);

    % 加速度
    xprimprim = 0;
    yprimprim = -a*a*c*m*m*sin(a*x+b);
    aG = [xprimprim; yprimprim];
    aL = to_R2d(theta)'*aG;
    
    motiondata = MotionData(t_time);
    motiondata.setT(x, y);
    motiondata.setPose(theta);
    motiondata.setVelocity(xprim, yprim);
    motiondata.setAcc(aL(1), aL(2));
    motiondata.setGyro(thetaprim);
    motiondatas = cat(1, motiondatas, motiondata);
    t_time = param.imu_timestep + t_time;
end

visualize(motiondatas);
save('sinmotiondata','motiondatas');
data2d = transformate(motiondatas);
save('sinmotiondata2d','data2d');
%visulizemotions(motiondatas)