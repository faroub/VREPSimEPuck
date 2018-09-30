function [errc delta_phi_obs] = getDeltaPhiObsDynamics( clientID, vrep, errc, psh, phi, beta_1,beta_2,delta_theta, delta_t, tau_t)
% compute the rate of change of the repulsive forcelets for the robot heading direction

global ROBOT_DISTANCE_BETWEEN_WHEELS ROBOT_PROXIMITY_SENSORS_DIRECTIONS MM2M


% compute distances to obstacles
[errc, detectionState, dist_obs] = getDistanceToObstacle(clientID,vrep,errc,psh);

% global angles of obstacles
psi_obs =(ROBOT_PROXIMITY_SENSORS_DIRECTIONS+ phi); % rad

% strength of repulsion
lambda_obs =  beta_1 * exp( -dist_obs/ beta_2 );

% double angle range of obstacle influence
sigma_obs = atan( tan( delta_theta / 2 ) + (((ROBOT_DISTANCE_BETWEEN_WHEELS/2))./ ( dist_obs + (ROBOT_DISTANCE_BETWEEN_WHEELS/2) )));


% relative angles of obstacles
zeta = ( phi - psi_obs );

% rate of change
delta_phi_obs =  lambda_obs .* zeta .* exp((-zeta.^2)./ (2 * (sigma_obs.^2)));

% euler solution
delta_phi_obs = (delta_t/tau_t) * delta_phi_obs;

% sum repulsive forcelets
delta_phi_obs = sum(delta_phi_obs);

end