
p_t = transpose([0 3 1]); % target end effector coordinate
Theta_0 = transpose([pi/4 0 pi/4 1]); % initial joint space values of the robot arm

vis = 1; % visualization indicator

alpha = 0.01; % optimization rate
epsilon = 0.005; % optimization error tolerance
p_0 = transpose([0 0 0]); % robot arm base origin


[ Theta_f, loss ] = RobotArmPlan( p_t, Theta_0, vis, alpha, epsilon, p_0 );


fprintf('Joint Angles and Pismatic Distances:\n');
fprintf('- theta_1 = %f\n', Theta_f(1,1));
fprintf('- theta_2 = %f\n', Theta_f(2,1));
fprintf('- theta_3 = %f\n', Theta_f(3,1));
fprintf('- d_4     = %f\n', Theta_f(4,1));
fprintf('\n');
fprintf('loss = %f\n', loss);
fprintf('\n');

