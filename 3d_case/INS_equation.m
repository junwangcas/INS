function [] = INS_equation(X, acc, gyro)
R_1_G = toR(1, X(1:3));
R_L = toR(1, gyro);
R_2_G = R_1_G*R_L;
% R to euler;



end

