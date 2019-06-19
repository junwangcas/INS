r = 0.1;
p = 0.2;
y = 0.3;
R = toR(0, r, p, y);
[r_test, p_test, y_test] = Rtorpy(R);
if abs(r-r_test) < 1e-4
    disp('roll correct');
end
if abs(p - p_test) < 1e-4
    disp('pitch correct');
end
if abs(y- y_test) < 1e-4
    disp('yaw correct');
end

% compared with matlab built in function
R_mat = eul2rotm([y, p, r]);
R - R_mat