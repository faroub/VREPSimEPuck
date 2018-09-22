% Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
% marc@coppeliarobotics.com
% www.coppeliarobotics.com
% 
% -------------------------------------------------------------------
% This file is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% 
% You are free to use/modify/distribute this file for whatever purpose!
% -------------------------------------------------------------------
%
% This file was automatically created for V-REP release V3.0.5 on October 26th 2013

% Make sure to have the server side running in V-REP!
% Start the server from a child script with following command:
%simExtRemoteApiStart(19999) % -- starts a remote API server service on port 19999

function moveEpuck()
clc
clear all
	disp('Program started');
    
    % initialize direction to target
    psi_tar = 0;
    % auxilary variables
    psi_tar_dot_aux = 0;
    psi_tar_aux = 0;
    
    %----- initialize state variables ---------------------------------
    % robot state
    robot_pos = [0;0]; % mm (should be integer for the simulator in : 0<x&y<600)
    robot_orient = 0; % rad
    robot_velocity = 0; % rad/s
    % initialize robot position
    pos = robot_pos; % mm
    % initialize robot heading direction
    phi = robot_orient; % rad
    % heading direction overall rate of change 
    delta_phi = 0; % rad /s
    % rate of change of attractive forcelet
    delta_phi_tar = 0; % rad/s
    % rate of change of repulsive forcelet
    delta_phi_obs = 0; % rad/s
    
    %----- attractive forcelet parameters ---
    % strength of attraction
    lambda_tar = 0.2;
    
    %----- repulsive forcelet parameters ----
    % maximum repulsion strength
    beta1_obs = 19;
    % spatial rate of decay
    beta2_obs = 30;
    
    % step time
    delta_t = 0.02;
    % time constant
    tau_t = 0.06;%0.067;
    
    encoders_old = [];
    encoders_new = [];
   
    ROBOT_WHEEL_RADUIS=21; % mm
    
    % set target state
    target_pos = [300;550];
    
    
    phiv=0;
    
 


         
       % where the program should be. there sould be a loop
       % --- run the program until simulation will be stopped
        while (1)           
   
              

              
              % current robot orinetation
              phi = phi + delta_phi % rad


              % normalize the orientation in the range -pi...pi (to be checked !)
              %phi = normalizeAngle(phi);
       

              

              
              % compute the direction and distance to target
              [psi_tar, d_tar] = getPsi(robot_pos, target_pos)
                 

             
              % compute rate of change of attractive forcelet   
              delta_phi_tar = getDeltaPhiTarDynamics(psi_tar, phi, lambda_tar, delta_t, tau_t);
              % compute rate of change of repulsive forcelet   
              % [delta_phi_obs dist_obs] = getDeltaPhiObsDynamics(phi, beta1_obs,beta2_obs, delta_t, tau_t);

                 % compute heading direction overall rate of change 
                 delta_phi = delta_phi_tar;% + delta_phi_obs;




                 %vr = vr + robot_velocity;
                 %vl = vl + robot_velocity;

       
       end
phiv
	
	disp('Program ended');
end
 