function [data2d] = transformate(motiondatas)
size_motion = length(motiondatas);
xs = zeros(size_motion,1);
ys = zeros(size_motion,1);
axs = zeros(size_motion,1);
ays = zeros(size_motion,1);
vxs = zeros(size_motion,1);
vys = zeros(size_motion,1);
thetas = zeros(size_motion,1);
ts = zeros(size_motion,1);
yawrates = zeros(size_motion,1);
for i = 1:size_motion
    motiondata = motiondatas(i);
    ts(i,1) = motiondata.t_time;
    xs(i,1) = motiondata.x;
    ys(i,1) = motiondata.y;  
    axs(i,1) = motiondata.ax;
    ays(i,1) = motiondata.ay;
    vxs(i,1) = motiondata.vx;
    vys(i,1) = motiondata.vy;
    thetas(i,1) = motiondata.theta;
    yawrates(i,1) = motiondata.yawrate;
end

data2d.IMU.t = ts;
data2d.IMU.acc = [axs';ays'];
data2d.IMU.gyro = yawrates';
data2d.GPS.pos_EN = [xs';ys'];
end

