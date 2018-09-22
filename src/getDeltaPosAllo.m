 function [enc_old, delta_pos_allo, delta_phi] = getDeltaPosAllo(enc_old,enc_new, phi_allo)
% compute the delta displacements in the allocentric coordinate frame

% get delta displacement in egocentric coordinate frame
[enc_old, delta_pos_ego, delta_phi]=getDeltaPosEgo(enc_old,enc_new);
% get delta displacement in allocentric coordinate frame
delta_pos_allo(1) = delta_pos_ego(1)*cos(phi_allo) - delta_pos_ego(2)*sin(phi_allo); % mm
delta_pos_allo(2) = delta_pos_ego(1)*sin(phi_allo) + delta_pos_ego(2)*cos(phi_allo); % mm
end