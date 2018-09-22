function [ vr, vl ] = getRotationVelocity(delta_phi, delta_t)
% compute the right and left wheels velocities to produce the necessary heading direction

global ROBOT_WHEEL_RADUIS ROBOT_DISTANCE_BETWEEN_WHEELS

% arc lengths
br = (delta_phi) * (ROBOT_DISTANCE_BETWEEN_WHEELS/2); % mm
bl = -(delta_phi) * (ROBOT_DISTANCE_BETWEEN_WHEELS/2); % mm

br=br/ROBOT_WHEEL_RADUIS;
bl=bl/ROBOT_WHEEL_RADUIS;

% right/left wheels velocities
vr = br / delta_t; % rad/s
vl = bl / delta_t; % rad/s



