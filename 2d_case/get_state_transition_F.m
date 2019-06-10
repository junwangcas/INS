function [fx, F] = get_state_transition_F(delta_t, yawrate,a_L,theta_t_1,v_t_1,s_t_1)
theta_t = theta_t_1 + (yawrate)*delta_t;
theta_t = normalize_theta(theta_t);
v_t = v_t_1 + to_R2d(theta_t_1)*a_L*delta_t;
s_t = s_t_1 + v_t_1*delta_t+ 1/2*to_R2d(theta_t_1)*a_L*delta_t*delta_t;
fx =[theta_t;v_t;s_t];

dR_theta = [-sin(theta_t_1),-cos(theta_t_1);cos(theta_t_1),-sin(theta_t_1)];
F = eye(5,5);
F(2:3, 1) = dR_theta*a_L*delta_t;
F(4:5, 1) =1/2*dR_theta*a_L*delta_t*delta_t;
F(4:5, 2:3) = eye(2,2)*delta_t;
end

