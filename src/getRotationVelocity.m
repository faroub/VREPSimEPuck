function [ vr, vl ] = getRotationVelocity(delta_phi, delta_t)
% compute the right and left wheels velocities to produce the necessary heading direction

global ROBOT_WHEEL_RADUIS ROBOT_DISTANCE_BETWEEN_WHEELS


% arc lengths
arcr = (delta_phi) * (ROBOT_DISTANCE_BETWEEN_WHEELS/2); % m
arcl = -(delta_phi) * (ROBOT_DISTANCE_BETWEEN_WHEELS/2); % m

% arc angles
angr=arcr/ROBOT_WHEEL_RADUIS; % rad
angl=arcl/ROBOT_WHEEL_RADUIS; % rad

% right/left wheels velocities
vr = angr / delta_t; % rad/s
vl = angl / delta_t; % rad/s



