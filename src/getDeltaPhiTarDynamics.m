function [delta_phi_tar, delta_phi_tar_range] = getDeltaPhiTarDynamics(psi_tar, phi, lambda, plot_range, delta_phi_tar_range, plot_dynamics, delta_t, tau_t)
% compute the rate of change of the attractor forcelet for the robot heading direction

% rate of change
% non-linear dynamics
delta_phi_tar = -lambda * sin(phi - psi_tar);

% prepare data for plot dynamics
if plot_dynamics==1
    delta_phi_tar_range = -lambda * sin(plot_range - psi_tar);
end

% euler solution
delta_phi_tar = (delta_t/tau_t) * delta_phi_tar;

end