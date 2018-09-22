function [psi, d] = getPsi(robot_pos, target_pos)
% compute the angle and distance to the target

% distance to target
d = sqrt((target_pos(1)-robot_pos(1))^2+(target_pos(2)-robot_pos(2))^2); % mm 
% direction to target
psi = atan2((target_pos(2)-robot_pos(2)),(target_pos(1)-robot_pos(1))); % rad
% normalize the direction in the range -pi...pi
% psi = normalizeAngle(psi);
end