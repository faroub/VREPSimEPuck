function [delta_phi_tar] = getDeltaPhiTarDynamics(psi_tar, phi, lambda, delta_t, tau_t)
% compute the rate of change of the attractor forcelet for the robot heading direction

% rate of change
% linear dynamics
%delta_phi_tar = -lambda * (phi - psi_tar);
% non-linear dynamics
delta_phi_tar = -lambda * sin(phi - psi_tar);

% euler solution
delta_phi_tar = (delta_t/tau_t) * delta_phi_tar;

end