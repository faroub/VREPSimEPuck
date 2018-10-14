 function [enc_old, delta_pos_allo, delta_phi] = getDeltaPosAllo(enc_old,enc_new, phi_allo)
% compute the delta displacements in the allocentric coordinate frame
global ROBOT_DISTANCE_BETWEEN_WHEELS

% compute delta encoders displacement
delta_enc = enc_new-enc_old; % m

% save the new reading as the old reading
enc_old = enc_new; % m

% get delta displacement in allocentric coordinate frame
delta_phi= (delta_enc(2)-delta_enc(1))/ROBOT_DISTANCE_BETWEEN_WHEELS; % rad
delta_pos_allo(1)=((delta_enc(2)+delta_enc(1))/2)*cos(phi_allo+(delta_phi/2)); % m
delta_pos_allo(2)=((delta_enc(2)+delta_enc(1))/2)*sin(phi_allo+(delta_phi/2)); % m





end