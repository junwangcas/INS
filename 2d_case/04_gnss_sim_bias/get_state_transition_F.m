function [fx, F] = get_state_transition_F(delta_t, yawrate,a_L, X)
s_t_1 = X(1:2);
v_t_1 = X(3:4);
theta_t_1 = X(5);
a_b = X(6:7);
theta_b = X(8);
s_t = s_t_1 + v_t_1*delta_t+ 1/2*to_R2d(theta_t_1)*(a_L + a_b)*delta_t*delta_t;
v_t = v_t_1 + to_R2d(theta_t_1)*(a_L + a_b)*delta_t;
theta_t = theta_t_1 + (yawrate + theta_b)*delta_t;
theta_t = normalize_theta(theta_t);


fx =[s_t; v_t; theta_t; a_b; theta_b]; 

F = zeros(8,8);
R = to_R2d(theta_t_1);
Rprim = to_R2dprim(theta_t_1);
% 1-2;
prim_p_theta = 0.5*Rprim*(a_L + a_b)*delta_t*delta_t;
prim_p_ab = 0.5*R*[1;1]*delta_t*delta_t;
F(1:2,1:2) = eye(2,2);
F(1:2,3:4) = eye(2,2)*delta_t;
F(1:2,5) = prim_p_theta;
F(1:2,6:7) = prim_p_ab;

end

