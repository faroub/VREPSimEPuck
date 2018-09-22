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
% change error code returned -> my be one variable is sufficient (todo)
% write Readme file (todo)
% complete the copyright and info part (todo)

function moveEpuck()
    clc
    clear all
	disp('Program started');

    % call constants into workspace
    mathConstants
    robotConstants
    
    %----- attractive forcelet parameters ---
    % strength of attraction
    lambda_tar = 0.5;
    
    %----- repulsive forcelets parameters ----
    % overall repulsion strength
    beta1_obs = 19;
    % spatial rate of decay
    beta2_obs = 30;
    % range of the force-let sensor sector
    delta_theta = 43*DEG2RAD; % rad
    
    %----- simulation parameters ----
    % step time
    delta_t = 0.05;
    % time constant
    tau_t = 1;
    
    %----- robot's kinematics parameters ----
    % robot velocity
    rob_vel=0.6;
    
    % using the prototype file (remoteApiProto.m)
    vrep=remApi('remoteApi'); 
    
    % just in case, close all opened connections
    vrep.simxFinish(-1); 
    
    % starts a communication thread with the server (i.e. V-REP)
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
  
    % create a clean up object
    cleanupObj = onCleanup(@()cleanMeUp(clientID,vrep));

    if (clientID>-1)
            
        disp('Connected to remote API server');   
            
        % start vrep simulation from Matlab script
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
        
        % retrieve robot handle 
        errc = zeros(1,25);
        [errc(1),rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
        while (errc(1)~=vrep.simx_error_noerror)  
            [errc(1),rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
        end

        % get robot position
        [errc(2), rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_blocking);
        rob_pos= rob_pos_vrep(:,1:2)*M2MM;
        while (errc(2)~=vrep.simx_error_noerror)  
            [errc(2), rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_blocking);
            rob_pos= rob_pos_vrep(:,1:2)*M2MM;
        end

        % get robot orientation
        [errc(3), rob_ori_vrep]=vrep.simxGetObjectOrientation(clientID,rh,-1,vrep.simx_opmode_blocking);
        rob_ori= rob_ori_vrep(3);
        while (errc(3)~=vrep.simx_error_noerror)  
            [errc(3), rob_ori_vrep]=vrep.simxGetObjectOrientation(clientID,rh,-1,vrep.simx_opmode_blocking);
            rob_ori= rob_ori_vrep(3);
        end

        % retrieve target handle
        [errc(4), th]=vrep.simxGetObjectHandle(clientID,'tarPos',vrep.simx_opmode_blocking);
        while (errc(4)~=vrep.simx_error_noerror)  
            [errc(4), th]=vrep.simxGetObjectHandle(clientID,'tarPos',vrep.simx_opmode_blocking);
        end

        % get target position
        [errc(5), tar_pos_vrep]=vrep.simxGetObjectPosition(clientID,th,-1,vrep.simx_opmode_blocking);
        tar_pos= tar_pos_vrep(:,1:2)*M2MM; % mm
        while (errc(5)~=vrep.simx_error_noerror)  
            [errc(5), tar_pos_vrep]=vrep.simxGetObjectPosition(clientID,th,-1,vrep.simx_opmode_blocking);
            tar_pos= tar_pos_vrep(:,1:2)*M2MM; % mm
        end


        % retrieve left joint handle
        [errc(6),jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);
        while (errc(6)~=vrep.simx_error_noerror)  
            [errc(6),jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);
        end

        % get right joint handle
        [errc(7),jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
        while (errc(7)~=vrep.simx_error_noerror)  
            [errc(7),jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
        end 

        % get left joint initial encoder value           
        [errc(8), jp(1)]=vrep.simxGetJointPosition(clientID, jh(1), vrep.simx_opmode_blocking);
        enc_old(1)  =jp(1)*ROBOT_WHEEL_RADUIS; % mm
        while (errc(8)~=vrep.simx_error_noerror)  
            [errc(8), jp(1)]=vrep.simxGetJointPosition(clientID, jh(1), vrep.simx_opmode_blocking);
            enc_old(1)  =jp(1)*ROBOT_WHEEL_RADUIS; % mm
        end  


       % get right joint initial encoder value       
        [errc(9), jp(2)]=vrep.simxGetJointPosition(clientID, jh(2), vrep.simx_opmode_blocking);
        enc_old(2)  =jp(2)*ROBOT_WHEEL_RADUIS; % mm
        while (errc(9)~=vrep.simx_error_noerror)  
            [errc(9), jp(2)]=vrep.simxGetJointPosition(clientID, jh(2), vrep.simx_opmode_blocking);
            enc_old(2)  =jp(2)*ROBOT_WHEEL_RADUIS; % mm 
        end
        
        % retrieve proximity sensors handles 
        psh = zeros(1, 8);
        for i = 10:17
            [errc(i),psh(i-9)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','ePuck_proxSensor',i-9),vrep.simx_opmode_blocking);
            while (errc(i)~=vrep.simx_error_noerror)
                [errc(i),psh(i-9)]=vrep.simxGetObjectHandle(clientID,sprintf('%s%d','ePuck_proxSensor',i-9),vrep.simx_opmode_blocking);
            end
        end

        % set robot initial orientation to 0
       vrep.simxSetObjectOrientation(clientID,rh,-1,[0,0,0],vrep.simx_opmode_oneshot);

        % set robot initial target velocity to 0
        vrep.simxSetJointTargetVelocity(clientID,jh(1),0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,jh(2),0,vrep.simx_opmode_oneshot);

            
            
        while (vrep.simxGetConnectionId(clientID)>-1)           

            % get left joint encoder value           
            [errc(8), jp(1)]=vrep.simxGetJointPosition(clientID, jh(1), vrep.simx_opmode_blocking); 
            enc_new(1)  = jp(1)*ROBOT_WHEEL_RADUIS; % mm 

            % get right joint initial encoder value       
            [errc(9), jp(2)]=vrep.simxGetJointPosition(clientID, jh(2), vrep.simx_opmode_blocking); 
            enc_new(2)  = jp(2)*ROBOT_WHEEL_RADUIS; % mm

            % get target position
            [errc(5), tar_pos_vrep]=vrep.simxGetObjectPosition(clientID,th,-1,vrep.simx_opmode_blocking);
            tar_pos= tar_pos_vrep(:,1:2)*M2MM; % mm

            % get delta displacements feedback  (position and orientation in the allocentric (world) coordinates frame) 
            [enc_old, delta_pos, delta_phi] = getDeltaPosAllo(enc_old, enc_new, rob_ori);

            % current robot orinetation
            rob_ori = rob_ori + delta_phi; % rad

            % current robot position
            rob_pos = rob_pos + delta_pos; % mm


            % compute the direction and distance to target
            [psi_tar, d_tar] = getPsi(rob_pos, tar_pos);

            % compute rate of change of attractive forcelet   
            delta_phi_tar = getDeltaPhiTarDynamics(psi_tar, rob_ori, lambda_tar, delta_t, tau_t);
            
            % compute distances to obstacles
            [errc, detectionState, dist_obs] = getDistanceToObstacle(clientID,vrep,errc,psh);
            
            % compute rate of change of repulsive forcelet 
            [delta_phi_obs dist_obs] = getDeltaPhiObsDynamics(rob_ori, dist_obs, beta1_obs,beta2_obs, delta_theta, delta_t, tau_t);

            % compute heading direction overall rate of change 
            delta_phi = delta_phi_tar;% + delta_phi_obs;


             % compute right and left wheels speeds
             [ vr, vl ] = getRotationVelocity(delta_phi, delta_t);
             vr = vr + rob_vel;
             vl = vl + rob_vel;

             % send commands to the robot
             vrep.simxSetJointTargetVelocity(clientID,jh(1),vl,vrep.simx_opmode_streaming);			
             vrep.simxSetJointTargetVelocity(clientID,jh(2),vr,vrep.simx_opmode_streaming);

             
             
         end

    else
		disp('Failed connecting to remote API server');
    end

end
 