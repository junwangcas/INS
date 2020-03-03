function [ Q ] = get_Q( input_args )
ratio_p = 0.5;
ratio_v = 0.5;
ratio_q = 0.1;
ratio_a_bias = 0.01;
ratio_theta_bias = 0.001;
Q = eye(8,8);
Q(1:2,1:2) = ratio_p*eye(2,2);
Q(3:4,3:4) = ratio_v*eye(2,2);
Q(5,5) = ratio_q;
Q(6:7,6:7) = ratio_a_bias*eye(2,2);
Q(8, 8) = ratio_theta_bias;
end

