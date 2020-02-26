load('../data/GNSSaidedINS_data2d.mat');
it_gps = 0;
scatter(data2d.GNSS.pos_EN(1,:),data2d.GNSS.pos_EN(2,:));
while (it_gps + 1 <= length(data2d.GNSS.t))
    it_gps
    it_gps = it_gps + 1;
    hold on;scatter(data2d.GNSS.pos_EN(1,1:it_gps),data2d.GNSS.pos_EN(2,1:it_gps),'+','r');
    xlabel('x-E');ylabel('y-north');
    if (mod(it_gps, 2) == 0)
        data2d.GNSS.t(it_gps)
        waitforbuttonpress;
    end
end