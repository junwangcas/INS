function [ Q ] = get_Q( input_args )
q_theta = 0.01;
v_theta = 0.05;
s_theta = 0.05;
Q = eye(5,5);
Q(1,1) = q_theta;
Q(2:3,2:3) = v_theta*eye(2,2);
Q(4:5,4:5) = s_theta*eye(2,2);
Q = Q*100;
end

