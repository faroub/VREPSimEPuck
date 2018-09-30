function [encoders_old, delta_pos_ego, delta_phi]=getDeltaPosEgo(encoders_old,encoders_new)
% compute the delta displacements in the egocentric coordinate frame

global ROBOT_DISTANCE_BETWEEN_WHEELS

% compute delta encoders displacement
delta_encoders = encoders_new-encoders_old; % m

% save the new reading as the old reading
encoders_old = encoders_new; % m

%straight forward movement
if delta_encoders(2)==delta_encoders(1) % m
    delta_pos_ego(1) = delta_encoders(1); % m
    delta_pos_ego(2) = 0; % m
    delta_phi = 0; % m     
%rotation on the spot    
elseif delta_encoders(2)==-delta_encoders(1) % m
    delta_pos_ego(1) = 0; % m
    delta_pos_ego(2) = 0; % m
    delta_phi = delta_encoders(2)/(0.5*ROBOT_DISTANCE_BETWEEN_WHEELS); % rad 
%rotation around icc    
else    
    b=0.5*(delta_encoders(2)+delta_encoders(1)); % m
    r=0.5*ROBOT_DISTANCE_BETWEEN_WHEELS*((delta_encoders(2)+delta_encoders(1))/(delta_encoders(2)-delta_encoders(1))); % m        
    delta_phi = b/r; % rad    
    delta_pos_ego(1) = r*sin(delta_phi); % m
    delta_pos_ego(2) = r*(1-cos(delta_phi)); % m 
end

  
end

