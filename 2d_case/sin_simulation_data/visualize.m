function [outputArg1,outputArg2] = visualize(motiondatas)
% for x/y
size_motion = length(motiondatas);
xs = zeros(size_motion,1);
ys = zeros(size_motion,1);
axs = zeros(size_motion,1);
ays = zeros(size_motion,1);
vxs = zeros(size_motion,1);
vys = zeros(size_motion,1);
thetas = zeros(size_motion,1);
for i = 1:size_motion
    motiondata = motiondatas(i);
    xs(i,1) = motiondata.x;
    ys(i,1) = motiondata.y;  
    axs(i,1) = motiondata.ax;
    ays(i,1) = motiondata.ay;
    vxs(i,1) = motiondata.vx;
    vys(i,1) = motiondata.vy;
    thetas(i,1) = motiondata.theta;
end
figure;
subplot(2,1,1);
scatter(xs,ys);
title('trajectory');
subplot(2,1,2);
plot(thetas);
title('theta');
% acc ax, ay;
figure;
subplot(2,1,1);
plot(axs);
title('accelerate x');
subplot(2,1,2);
plot(ays);
title('accelerate y');
% speed;
figure;
subplot(2,1,1);
plot(vxs);
subplot(2,1,2);
plot(vys);
title('speed');




end

