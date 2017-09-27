function [ Theta_f, loss ] = RobotArmPlan( p_t, Theta_0, vis, alpha, epsilon, p_0 )
% Input:
% - p_t Target end effector coordinate.
% - Theta_0 Initial joint space values of the robot arm.
% - vis Should visualize intermediate and final steps. (optional, default: 0)
% - alpha Optimization rate. (optional, default: 0.01)
% - epsilon Optimization error tolerance. (optional, default: 0.005)
% - p_0 Robot arm base origin. (optional, default: [0 0 0])
%
% Output:
% - Theta_f The finalized joint space value.
% - loss The final loss.

syms theta_1 theta_2 theta_3 d_4

Theta = transpose([theta_1 theta_2 theta_3 d_4]); % joint space values
if ~exist('vis', 'var')
    vis = 0; % visualization indicator
end
if ~exist('p_0', 'var')
    p_0 = transpose([0 0 0]); % robot arm base origin
end
if ~exist('alpha', 'var')
    alpha = 0.01; % optimization rate
end
if ~exist('epsilon', 'var')
    epsilon = 0.005; % optimization error tolerance
end


[ p, H ] = RobotArmFK(Theta, p_0);

l = RobotArmLoss(p_t, p);
dl_dTheta = jacobian(l, Theta);

Theta_t = Theta_0;
if vis
    fprintf('Planning and Optimization:\n');
end
for t = 1:100
    subs_l = double(subs(l, Theta, Theta_t));
    
    if vis
        fprintf('[ %d ] loss = %f\n', t, subs_l) % print the instant optimization step and instant loss
        hold on;
        RobotArmVisualize(Theta_t); % visualize instant intermediate pose
        drawnow;
        pause(0.2);
    end
    
    if subs_l < epsilon
        break
    end
    
    subs_dl_dTheta = double(subs(dl_dTheta, Theta, Theta_t));
    Theta_t = Theta_t - alpha * transpose(subs_dl_dTheta);
end

subs_l = double(subs(l, Theta, Theta_t));

if vis
    fprintf('\n')
    fprintf('Planning and Optimization Summary:\n');
    fprintf('- total optimization steps = %d\n', t);
    fprintf('- final optimization loss: %f\n', subs_l);
    fprintf('\n');
    hold on;
    RobotArmVisualize(Theta_t); % visualize finalized pose
end

Theta_f = Theta_t;
loss = subs_l;

end
