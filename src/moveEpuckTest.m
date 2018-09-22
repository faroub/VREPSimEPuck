%%%%%%%%%%%%%%%%%%%% Vrep Simulator Interface Class %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author  :  Farid Oubbati 
%   email   :   f.oubbati@yahoo.fr
%   Date    :   August 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Developed as a part of FIRM Toolbox for Matlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% build borders on the scene (todo)
% write Readme file (todo)
% implement a mechanism to terminate matlab script when simulation stopped by user (todo)
% complete the copyright and info part (todo)

function moveEpuck()
    clc
    clear all

	disp('Program started');
    
    % Constants
    ROBOT_WHEEL_RADUIS=21; % mm -> epuck wheel radius
    m2mm=1000;
    mm2m=1/1000;
    rad2deg=180/pi;
    deg2rad=pi/180;
    
    % Variables
    % initialize direction to target
    psi_tar = 0;
    % auxilary variables
    psi_tar_dot_aux = 0;
    psi_tar_aux = 0;
    
    %----- initialize state variables ---------------------------------
    % robot state
    %rob_pos = [0;0]; % mm (should be integer for the simulator in : 0<x&y<600)
    rob_ori = 0; % rad
    rob_vel = 0; % rad/s
    % initialize robot position
    %pos = rob_pos; % mm
    % initialize robot heading direction
    phi = rob_ori; % rad
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
    

   rob_vel=0.2;
    
    
    % set target state
    %tar_pos = [300;550];
    
    
    
    
 
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
  
    % create clean up object
    cleanupObj = onCleanup(@()cleanMeUp(vrep, clientID));


        if (clientID>-1)
            disp('Connected to remote API server');   
            
            % start vrep simulation from Matlab script
            vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);

           % retrieve robot handle            
           [errc(1),rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_oneshot_wait);
           while (errc(1)~=vrep.simx_error_noerror)  
                [errc(1),rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_oneshot_wait);
           end
            
           % get robot position
           [errc(2), rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_oneshot_wait);
           rob_pos= rob_pos_vrep(:,1:2)*m2mm;
           while (errc(2)~=vrep.simx_error_noerror)  
                [errc(2), rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_oneshot_wait);
                rob_pos= rob_pos_vrep(:,1:2)*m2mm;
           end
           
           % get robot orientation
           [errc(3), rob_ori_vrep]=vrep.simxGetObjectOrientation(clientID,rh,-1,vrep.simx_opmode_oneshot_wait);
           rob_ori= rob_ori_vrep(3);
           while (errc(3)~=vrep.simx_error_noerror)  
                [errc(3), rob_ori_vrep]=vrep.simxGetObjectOrientation(clientID,rh,-1,vrep.simx_opmode_oneshot_wait);
                rob_ori= rob_ori_vrep(3);
           end
           
           
           % retrieve target handle
           [errc(4), th]=vrep.simxGetObjectHandle(clientID,'tarPos',vrep.simx_opmode_oneshot_wait);
           while (errc(4)~=vrep.simx_error_noerror)  
                [errc(4), th]=vrep.simxGetObjectHandle(clientID,'tarPos',vrep.simx_opmode_oneshot_wait);
           end
           
           % get target position
           [errc(5), tar_pos_vrep]=vrep.simxGetObjectPosition(clientID,th,-1,vrep.simx_opmode_oneshot_wait);
           tar_pos= tar_pos_vrep(:,1:2)*m2mm;
           while (errc(5)~=vrep.simx_error_noerror)  
                [errc(5), tar_pos_vrep]=vrep.simxGetObjectPosition(clientID,th,-1,vrep.simx_opmode_oneshot_wait);
                tar_pos= tar_pos_vrep(:,1:2)*m2mm;
           end
           
            % get wheels objects handles
            % left joint
            [errc(6),jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_oneshot_wait);
            while (errc(6)~=vrep.simx_error_noerror)  
                [errc(6),jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_oneshot_wait);
            end
            
            % right joint
           [errc(7),jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_oneshot_wait);
           while (errc(7)~=vrep.simx_error_noerror)  
                [errc(7),jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_oneshot_wait);
           end 
           
                       
            % retrieve initial value of encoders
            % left joint
            if (errc(6)==vrep.simx_error_noerror)             
                [errc(8), jp(1)]=vrep.simxGetJointPosition(clientID, jh(1), vrep.simx_opmode_streaming);
                while (errc(8)~=vrep.simx_error_noerror)  
                    [errc(8), jp(1)]=vrep.simxGetJointPosition(clientID, jh(1), vrep.simx_opmode_streaming);
                    enc_old(1)  =jp(1)*ROBOT_WHEEL_RADUIS; % mm *ROBOT_WHEEL_RADUIS; % mm 

                end  
            end
            
            % right joint
            if (errc(7)==vrep.simx_error_noerror)             
                [errc(9), jp(2)]=vrep.simxGetJointPosition(clientID, jh(2), vrep.simx_opmode_streaming);
                while (errc(9)~=vrep.simx_error_noerror)  
                    [errc(9), jp(2)]=vrep.simxGetJointPosition(clientID, jh(2), vrep.simx_opmode_streaming);
                    enc_old(2)  =jp(2)*ROBOT_WHEEL_RADUIS; % mm *ROBOT_WHEEL_RADUIS; % mm 
                end
            end
            
            
            while (vrep.simxGetConnectionId(clientID)>-1)           
                %----- robot feedback ---------------------------------------------  
                
                % get wheels position
                % left joint
                [errc(8), jp(1)]=vrep.simxGetJointPosition(clientID, jh(1), vrep.simx_opmode_buffer); 
                enc_new(1)  =jp(1)*ROBOT_WHEEL_RADUIS; % mm *ROBOT_WHEEL_RADUIS; % mm 

                % right joint
                [errc(9), jp(2)]=vrep.simxGetJointPosition(clientID, jh(2), vrep.simx_opmode_buffer); 
                enc_new(2)  =jp(2)*ROBOT_WHEEL_RADUIS; % mm *ROBOT_WHEEL_RADUIS; % mm 

lw=jp(1)*rad2deg
rw=jp(2)*rad2deg
                 vr = 0.1;
                 vl = 0.1;
                 
                 % send commands to the robot
                 vrep.simxSetJointTargetVelocity(clientID,jh(1),vl,vrep.simx_opmode_streaming);			
                 vrep.simxSetJointTargetVelocity(clientID,jh(2),vr,vrep.simx_opmode_streaming);
                 
            end

    else
		disp('Failed connecting to remote API server');
    end
	disp('Program ended');
    
    % create clean up function
    function cleanMeUp(vrep, clientID)
        % stop vrep simulation from Matlab script
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
        vrep.simxFinish(clientID); % close the line if still open
        vrep.delete(); % explicitely call the destructor!
        disp('Program ended by user');
    end

end
 