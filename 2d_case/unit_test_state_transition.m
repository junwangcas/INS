%[fx, F] = get_state_transition_F(delta_t, yawrate,a_L,theta_t_1,v_t_1,s_t_1)

s_t_1 = [0;0];
v_t_1 = [0;0];
yawrate = 1;
delta_t = 1;
a_L = [1;1];
theta_t_1 = pi;

theta_t = theta_t_1 + yawrate*delta_t;
v_t = a_L*delta_t;
s_t = s_t_1 + 0.5*a_L*delta_t*delta_t;
fx_verify = [theta_t; v_t; s_t];

[fx,F] = get_state_transition_F(delta_t,yawrate, a_L, theta_t_1, v_t_1, s_t_1);
fx
