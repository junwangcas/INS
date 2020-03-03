function [theta_output] = normalize_theta(theta_input)
% theta should be [-pi, pi);
while theta_input < -pi
    theta_input = theta_input + 2*pi;
end
while theta_input >= pi
    theta_input = theta_input - 2*pi;
end
theta_output = theta_input;
end

