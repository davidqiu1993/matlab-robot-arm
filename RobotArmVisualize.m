function RobotArmVisualize( Theta, p_0 )
% Input:
% - Theta The joint space value vector.
%         Theta = [ theta_1 theta_2 theta_3 d_4 ]T
% - p_0 The origin of the robot arm.
%       p_0 = [ p_0_x p_0_y p_0_z ]T
%       p_0 = [ 0 0 0 ]T by default
% 
% Robot Arm Configurations:
% ---------------------------------------
% i  theta_i           d_i  a_i  alpha_i
% ---------------------------------------
% 1  theta_1           1    0    -pi/2
% 2  (theta_2 - pi/2)  0    3    0
% 3  theta_3           0    3    -pi/2
% 4  0                 d_4  0    0
% ---------------------------------------

theta_1 = Theta(1, 1);
theta_2 = Theta(2, 1);
theta_3 = Theta(3, 1);
d_4 = Theta(4, 1);

if ~exist('p_0', 'var')
    p_0 = transpose([0 0 0]);
end
p_0_x = p_0(1,1);
p_0_y = p_0(2,1);
p_0_z = p_0(3,1);
P_0 = transpose([p_0_x p_0_y p_0_z 1]);

H_0_1 = DHFrameTransform(theta_1,        1,   0, -pi/2 );
H_1_2 = DHFrameTransform(theta_2 - pi/2, 0,   3, 0     );
H_2_3 = DHFrameTransform(theta_3,        0,   3, -pi/2 );
H_3_4 = DHFrameTransform(0,              d_4, 0, 0     );

H_0_2 = H_0_1 * H_1_2;
H_0_3 = H_0_2 * H_2_3;
H_0_4 = H_0_3 * H_3_4;

P_1 = H_0_1 * P_0;
P_2 = H_0_2 * P_0;
P_3 = H_0_3 * P_0;
P_4 = H_0_4 * P_0;

PMat = [P_0 P_1 P_2 P_3 P_4];


figure(1);

plot3(PMat(1,:), PMat(2,:), PMat(3,:));
hold on;
plot3(PMat(1,:), PMat(2,:), PMat(3,:), 'o');

xlim([-7 7]);
ylim([-7 7]);
zlim([0 7]);
grid on;

end
