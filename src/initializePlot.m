function  [plot_range, phasePlotRange, phasePlot, targetPlot, delta_phi_tar_range, delta_phi_obs_range]=initializePlot(plot_dynamics)
% setup and initialize plot dynamics

    if plot_dynamics==0
        
        % default values to output arguments
        plot_range=[];
        phasePlotRange=[];
        phasePlot=[];
        targetPlot=[];
        delta_phi_tar_range=[];
        delta_phi_obs_range=[];
    
    
    else
        % plot range
        plot_range = -pi:pi/32:pi;
        delta_phi_tar_range = zeros(1, length(plot_range));
        delta_phi_obs_range = zeros(1, length(plot_range));

        % phi
        fig = figure( 'Name', 'plot of the dynamics', ...
                                             'MenuBar', 'none', ...
                                             'NumberTitle', 'off');
        phasePlotAxes=axes();
        phasePlotRange = plot( phasePlotAxes, plot_range, delta_phi_tar_range, 'g',... 
                                              plot_range, delta_phi_obs_range, 'r',...
                                              plot_range, delta_phi_tar_range + delta_phi_obs_range, 'b');


        set( phasePlotRange, 'LineWidth', 3 );
        hold on
        phasePlot = plot(phasePlotAxes,0,0,'o', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
        targetPlot = plot(phasePlotAxes,0,0,'x', 'MarkerFaceColor', 'c', 'MarkerSize', 8);
        set(phasePlotAxes,'XLim',[-pi pi],'YLim',[-5 5]);
        grid on;
        xlabel( '\phi' );
        ylabel( 'd\phi / dt' );
        legend({'attract dyn','repel dyn','comb dyn','current \phi','target \psi'},'Location','southwest')
        drawnow;
    end



end