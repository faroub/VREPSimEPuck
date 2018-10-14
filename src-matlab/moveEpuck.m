% Dynamical Obstacles Avoidance and Target Acquisition for Robot Navigation using the 
% Attractor Dynamics Approach 
% 
% This project demonstrates  that the dynamic system approach can be used to generate
% collision-free paths toward targets while avoid moving obstacles even if low-level sensory
% information (proximity sensors in this case) is used instead of representations of the
% environment, see (Althaus et al., 2001) or (Bicho et al., 1998) in docs folder for more information.
% 
% The simulation setup:
% The dynamics generate a heading direction for a mobile robot that is
% moving successively but in a random order toward three predefined  targets
% while avoiding randomly moving obstacles.
% 
% The simulation robot used:
% - ePuck
%
% The versions of simulation softwares used :
% - Matlab 8.5.0.197613 (R2015a) 
% - V-REP PRO EDU version 3.5.0
%
% To run the simulation:
% 1 - Start the V-REP simulator and open the simulation scene provided (epuck-dyn-obs-tar.ttt)
% 2 - Start Matlab and change to simulation files folder or add its path
% 3 - Start simulation with: 
%           >> moveEpuck : runs the simulation without plotting the dynamics
%           of the heading direction
%           >> moveEpuck(1) :  runs the simulation and plots the dynamics
%           of the heading direction
% 4 - If necessary, change the parameters of the dynamics and rerun the simulation  
% 
% These files are distributed in the hope that they will be useful,
% but without any warranty; without even the implied warranty of
% merchantability or fitness for a particular purpose.
%
% Please feel free to use/modify/distribute this file for whatever
% purpose!.
%
% Copyright 2017, Farid Oubbati 
% f.oubbati@yahoo.fr
% Date: 12-May-2017

function moveEpuck(plot_dynamics)
    
    % initialize workspace
    clc
    clear all
    close all
	disp('Program started');

    % plot_dynamics = set to 1 if dynamics should be plotted
    if nargin<1 plot_dynamics=0; end;
    
    % call constants into workspace
    mathConstants
    robotConstants
    
    %----- attractive forcelet parameters ---
    % strength of attraction
    lambda_tar =0.5;
    
    %----- repulsive forcelets parameters ----
    % overall repulsion strength
    beta1_obs = 10;
    % spatial rate of decay
    beta2_obs = 0.2;
    % range of the force-let sensor sector
    delta_theta = 40*DEG2RAD; % rad
    
    %----- simulation parameters ----
    % step time
    delta_t = 0.02;
    % time constant
    tau_t = 0.5;
    
    %----- robot's kinematics parameters ----
    % robot velocity
    rob_vel=2;
    
    %----- robot information ---
    % robot joints handles
    jh=zeros(1, 2);
    % robot position
    rob_pos=zeros(1, 2);
    % robot joints positions
    jp=zeros(1, 2);
    % robot encoders values
    enc_old=zeros(1, 2);
    enc_new=zeros(1, 2);
    % robot proximity sensors handles 
    psh = zeros(1, 8);

    %----- target information ----
    % target handles
    th=zeros(1, 3);
    % target positions
    tar_idx = 1;
    tar_positions= zeros(size(th,2)+1, 2);
    tar_nbr=4*ones(1, 4);
    
    %----- obstacles information ----
    % obstacle handles 
    obsh = zeros(1, 8);
    % obstacle handles 
    obs_positions= zeros(size(obsh,2), 5);
    % movement step size
    obs_move_step=0.005; % m
    
    %----- setup phase plot ----
    [plot_range, phasePlotRange, phasePlot, targetPlot, delta_phi_tar_range, delta_phi_obs_range]=initializePlot(plot_dynamics);
    
    % using the prototype file (remoteApiProto.m)
    vrep=remApi('remoteApi'); 
    
    % just in case, close all opened connections
    vrep.simxFinish(-1); 
    
    % starts a communication thread with the server (i.e. V-REP)  on port 19999
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
  
    % create a clean up object
    cleanupObj = onCleanup(@()cleanMeUp(clientID,vrep));

    if (clientID>-1)
            
        disp('Connected to remote API server');   

        % specify the simulation timestep
         vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, ...
             delta_t,  vrep.simx_opmode_oneshot);
        
         
        % retrieve robot handle 
        [err_code,rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
        while (err_code~=vrep.simx_error_noerror)  
            [err_code,rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
        end

        % get robot position
        [err_code, rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_blocking);
        rob_pos= rob_pos_vrep(1:2); % m
        while (err_code~=vrep.simx_error_noerror)  
            [err_code, rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_blocking);
            rob_pos= rob_pos_vrep(1:2); % m
        end
        
        % set base (initial robot) position
        tar_positions(4,1:2)=rob_pos;
        
        % get robot orientation
        [err_code, rob_ori_vrep]=vrep.simxGetObjectOrientation(clientID,rh,-1,vrep.simx_opmode_blocking);
        rob_ori= rob_ori_vrep(3); % rad
        while (err_code~=vrep.simx_error_noerror)  
            [err_code, rob_ori_vrep]=vrep.simxGetObjectOrientation(clientID,rh,-1,vrep.simx_opmode_blocking);
            rob_ori= rob_ori_vrep(3); % rad
        end

        % retrieve targets handles
        for i = 1:size(th,2)
            [err_code, th(i)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','tarPos',i),vrep.simx_opmode_blocking);
            while (err_code~=vrep.simx_error_noerror)  
                [err_code, th(i)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','tarPos',i),vrep.simx_opmode_blocking);
            end
        end

        % get targets positions
        for i = 1:size(th,2)
            [err_code, tar_pos_vrep]=vrep.simxGetObjectPosition(clientID,th(i),-1,vrep.simx_opmode_blocking);
            tar_positions(i,1:2)= tar_pos_vrep(1:2); % m
            while (err_code~=vrep.simx_error_noerror)  
                [err_code, tar_pos_vrep]=vrep.simxGetObjectPosition(clientID,th(i),-1,vrep.simx_opmode_blocking);
                tar_positions(i,1:2)= tar_pos_vrep(1:2); % m
            end
        end

        % retrieve left joint handle
        [err_code,jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);
        while (err_code~=vrep.simx_error_noerror)  
            [err_code,jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);
        end

        % get right joint handle
        [err_code,jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
        while (err_code~=vrep.simx_error_noerror)  
            [err_code,jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
        end 

        % get joints handles and initial encoders values 
        for i = 1:size(jh,2)
            % set initial joint initial encoder value
            [err_code]=vrep.simxSetJointPosition(clientID, jh(i), 0, vrep.simx_opmode_oneshot);
            while (err_code~=vrep.simx_error_noerror) 
                        [err_code]=vrep.simxSetJointPosition(clientID, jh(i), 0, vrep.simx_opmode_oneshot);
            end
            
            % get joints initial encoders values 
            [err_code, jp(i)]=vrep.simxGetJointPosition(clientID, jh(i), vrep.simx_opmode_streaming);
            enc_old(i)  =jp(i)*ROBOT_WHEEL_RADUIS; % m
            enc_new(i) =enc_old(i); % m
            while (err_code~=vrep.simx_error_noerror)  
                [err_code, jp(i)]=vrep.simxGetJointPosition(clientID, jh(i), vrep.simx_opmode_streaming);
                enc_old(i)  =jp(i)*ROBOT_WHEEL_RADUIS; % m
                enc_new(i) =enc_old(i); % m
            end  
        end

        % retrieve proximity sensors handles 
        for i = 1:size(psh,2)
            [err_code,psh(i)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','ePuck_proxSensor',i),vrep.simx_opmode_blocking);
            while (err_code~=vrep.simx_error_noerror)
                [err_code,psh(i)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','ePuck_proxSensor',i),vrep.simx_opmode_blocking);
            end
        end

        % retrieve obstacles handles 
        for i = 1:size(obsh,2)
            [err_code,obsh(i)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','obstPos',i),vrep.simx_opmode_blocking);
            while (err_code~=vrep.simx_error_noerror)
                [err_code,obsh(i)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','obstPos',i),vrep.simx_opmode_blocking);
            end
        end
        
        % get the obstacles current positions
        for i = 1:size(obsh,2)
            [err_code, obs_pos_vrep]=vrep.simxGetObjectPosition(clientID,obsh(i),-1,vrep.simx_opmode_blocking);
            obs_positions(i,1:3)= obs_pos_vrep; % m
            while (err_code~=vrep.simx_error_noerror)
                [err_code, obs_pos_vrep]=vrep.simxGetObjectPosition(clientID,obsh(i),-1,vrep.simx_opmode_blocking);
                obs_positions(i,1:3)= obs_pos_vrep; % m
            end
        end
        
        
        % set robot initial orientation to 0
       vrep.simxSetObjectOrientation(clientID,rh,-1,[0,0,0],vrep.simx_opmode_oneshot);

        % set robot initial target velocity to 0
        vrep.simxSetJointTargetVelocity(clientID,jh(1),0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,jh(2),0,vrep.simx_opmode_oneshot);
        

        % target selection
        tar_nbr(1,1:size(th,2))=randperm(size(th,2));
        tar_pos = tar_positions(tar_nbr(tar_idx),:);
        %tar_pos = tar_positions(2,:);

        % set obstacles movement behaviors
        obs_positions(1:size(obsh,2),4)=randi([1 8],1,size(obsh,2));
        % set obstacles movement steps
        obs_positions(1:size(obsh,2),5)= obs_move_step;
       
        % enable the synchronous mode    
        vrep.simxSynchronous(clientID,true);
        
        % start vrep simulation
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        
        disp('V-REP simulation started');
        
        while (vrep.simxGetConnectionId(clientID)>-1)           
            
            % get joint encoders values
            for i = 1:size(jh,2)
                [err_code, jp(i)]=vrep.simxGetJointPosition(clientID, jh(i), vrep.simx_opmode_buffer); 
                enc_new(i)  = jp(i)*ROBOT_WHEEL_RADUIS; % m 
            end
            
            % get delta displacements feedback  (position and orientation in the allocentric (world) coordinates frame) 
            [enc_old, delta_pos, delta_phi] = getDeltaPosAllo(enc_old, enc_new, rob_ori);

            % current robot orinetation
            rob_ori = rob_ori + delta_phi; % rad
            
            % normalize the orientation in the range -pi...pi
            rob_ori = normalizeAngle(rob_ori);
               
            % current robot position
            rob_pos = rob_pos + delta_pos; % m
          
            % compute the direction and distance to target
            [psi_tar, d_tar] = getPsi(rob_pos, tar_pos);


            % get target position
            [tar_pos, tar_idx ] = getTargetPosition( tar_positions, tar_pos, d_tar, tar_nbr, tar_idx);

  
            % compute rate of change of attractive forcelet   
            [delta_phi_tar, delta_phi_tar_range] = getDeltaPhiTarDynamics(psi_tar, rob_ori, lambda_tar, plot_range, delta_phi_tar_range, plot_dynamics, delta_t, tau_t);
            
                        
            % compute rate of change of repulsive forcelet 
            [delta_phi_obs, delta_phi_obs_range] = getDeltaPhiObsDynamics(clientID, vrep, psh, rob_ori, beta1_obs,beta2_obs, delta_theta, plot_range, delta_phi_obs_range, plot_dynamics, delta_t, tau_t);

            % compute heading direction overall rate of change 
            delta_phi = delta_phi_tar + delta_phi_obs;

 
             % compute right and left wheels speeds
             [ vr, vl ] = getRotationVelocity(delta_phi, delta_t);
             vr = vr + rob_vel;
             vl = vl + rob_vel;
             
             % send commands to the robot
             %vrep.simxPauseCommunication(clientID,1);
%               vrep.simxSetJointTargetVelocity(clientID,jh(1),vl,vrep.simx_opmode_streaming);			
%               vrep.simxSetJointTargetVelocity(clientID,jh(2),vr,vrep.simx_opmode_streaming);
             %vrep.simxPauseCommunication(clientID,0);
             
             if tar_idx > 4
                break
             end
             
             [obs_positions]=moveObstacles(clientID, vrep, obsh, obs_positions, obs_move_step);
             
             % plot dynamics
             plotDynamics(rob_ori, delta_phi, phasePlot, targetPlot, phasePlotRange, delta_phi_tar_range, delta_phi_obs_range, psi_tar, plot_dynamics);
             
             
             % move simulation ahead one time step
             vrep.simxSynchronousTrigger(clientID);
             vrep.simxGetPingTime(clientID);
            
            
                          
         end

    else
		disp('Failed connecting to remote API server');
    end

end
 