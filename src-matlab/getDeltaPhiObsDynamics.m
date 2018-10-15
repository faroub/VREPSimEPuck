function [ delta_phi_obs, delta_phi_obs_range] = getDeltaPhiObsDynamics( clientID, vrep, psh, phi, beta_1,beta_2, delta_theta, plot_range, delta_phi_obs_range, plot_dynamics, delta_t, tau_t)
% compute the rate of change of the repulsive forcelets for the robot heading direction

global ROBOT_DISTANCE_BETWEEN_WHEELS ROBOT_PROXIMITY_SENSORS_DIRECTIONS

% robot radius
rob_rad=ROBOT_DISTANCE_BETWEEN_WHEELS/2;

% compute distances to obstacles
dist_obs = getDistanceToObstacle(clientID,vrep,psh);

% global angles of obstacles
psi_obs =(ROBOT_PROXIMITY_SENSORS_DIRECTIONS+ phi); % rad

% strength of repulsion
lambda_obs =  beta_1 * exp( -beta_2*dist_obs / rob_rad );

% angular range of obstacles influence
sigma_obs = atan( (tan( delta_theta)/2) + (rob_rad./ ( dist_obs + rob_rad )));

% prepare data for plot dynamics
if plot_dynamics==1
    % dimensions variables
    n = length( ROBOT_PROXIMITY_SENSORS_DIRECTIONS );
    m = length( plot_range );
    % expand vectors to avoid loops
    lambda_obs_range = repmat( lambda_obs', 1, m );
    sigma_obs_range  = repmat( sigma_obs', 1, m );
    % calculate result relative to a given phi
    zeta_range = normalizeAngle( repmat( plot_range, n, 1 ) - repmat( psi_obs', 1, m ) );
    % rate of change
    delta_phi_obs_range = lambda_obs_range .* zeta_range .* exp(-(zeta_range.^2)./ (2 * (sigma_obs_range.^2)));
    % sum repulsive forcelets
    delta_phi_obs_range = sum(delta_phi_obs_range);
end

% relative angles of obstacles
zeta = normalizeAngle( phi - psi_obs );

% rate of change
delta_phi_obs =  lambda_obs .* zeta .* exp(-(zeta.^2)./ (2 * (sigma_obs.^2)));

% euler solution
delta_phi_obs = (delta_t/tau_t) * delta_phi_obs;

% sum repulsive forcelets
delta_phi_obs = sum(delta_phi_obs);

end