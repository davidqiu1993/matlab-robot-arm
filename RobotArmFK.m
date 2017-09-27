function [ p, H ] = RobotArmFK( Theta, p_0 )
% Input:
% - Theta The joint space value vector.
%         Theta = [ theta_1 theta_2 theta_3 d_4 ]T
% - p_0 The origin of the robot arm.
%       p_0 = [ p_0_x p_0_y p_0_z ]T
%       p_0 = [ 0 0 0 ]T by default
% 
% Output:
% - p The position of the end effector.
%     p = [ x y z ]T
% - H The homogeneous transformation matrix from the origin of the robot 
%     arm to the end effector.
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

H = H_0_1 * H_1_2 * H_2_3 * H_3_4;

P_1 = H * P_0;

p = [ P_1(1,1);
      P_1(2,1);
      P_1(3,1) ];

end
