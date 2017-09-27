function [ H ] = DHFrameTransform ( theta, d, a, alpha )

Rot_z_theta = [ cos(theta) (-sin(theta)) 0 0;
                sin(theta) cos(theta)    0 0;
                0          0             1 0;
                0          0             0 1 ];

Trans_z_d = [ 1 0 0 0;
              0 1 0 0;
              0 0 1 d;
              0 0 0 1 ];

Trans_x_a = [ 1 0 0 a;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1 ];

Rot_x_alpha = [ 1 0          0             0;
                0 cos(alpha) (-sin(alpha)) 0;
                0 sin(alpha) cos(alpha)    0;
                0 0          0             1 ];

H = Rot_z_theta * Trans_z_d * Trans_x_a * Rot_x_alpha;

end
