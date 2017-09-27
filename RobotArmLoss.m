function [ l ] = RobotArmLoss( p_t, p )
% Input:
% - p_t The target task space end effector coordinate.
% - p The current task space end effector coordinate.
%
% Output:
% - l The loss function or the cost function value.

l = transpose(p_t - p) * (p_t - p) / 2;

end
