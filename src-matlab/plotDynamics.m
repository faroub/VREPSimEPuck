function plotDynamics(phi, delta_phi, phasePlot, targetPlot, phasePlotRange, delta_phi_tar_range, delta_phi_obs_range, psi_tar, plot_dynamics)
% plot dynamics of the heading direction


if plot_dynamics==1
    
    set(phasePlot, 'XData', phi);
    set(phasePlot, 'YData', delta_phi);
    set(targetPlot, 'XData', psi_tar);
    set(phasePlotRange(1), 'YData', delta_phi_tar_range);
    set(phasePlotRange(2), 'YData', delta_phi_obs_range);
    set(phasePlotRange(3), 'YData', delta_phi_tar_range + delta_phi_obs_range);
    drawnow;

end