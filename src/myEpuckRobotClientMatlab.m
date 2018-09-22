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

function myEpuckRobotClientMatlab()
	disp('Program started');
    % initialize some variables
    %--------- perceptual field parameters --------------------------
    % field resting level
    h_d = -5; 
    % steepness of sigmoid output function
    beta_d = 5; 
    % inflection point of sigmoid output function
    x0_d = 0; 
    % excitation strength
    c_exc = 18;
    % inihibtion strength
    g_inh = 0.01;
    % input strength
    c_in = 5;
    % noise std
    q_d = 0.005; 
    % noise strength
    c_qd = 1; 

    % field variables
    % x dim
    % feature dimension x (heading direction)  
    metric_x_step = pi/64;
    metric_x_init = -pi;
    metric_x_final = pi;
    metric_x_dim = metric_x_init:metric_x_step:metric_x_final;
    field_x_perc_nodes=1:length(metric_x_dim);
    field_x_perc_size=length(metric_x_dim);
    field_x_perc_half_size = floor(field_x_perc_size/2);
    % gaussian interaction kernel with local excitation and global inhibition
    field_x_perc_kern=1 *...% strength of gaussian
    gaussNorm(-field_x_perc_half_size:field_x_perc_half_size,...
    0,... % mean value
    5);   % width of gaussian   

    % y dim
    % feature dimension y (hue value)  
    metric_y_step = 0.01;
    metric_y_init = 0;
    metric_y_final = 1;
    metric_y_dim = metric_y_init:metric_y_step:metric_y_final;
    field_y_perc_nodes=1:length(metric_y_dim);
    field_y_perc_size=length(metric_y_dim);
    field_y_perc_half_size = floor(field_y_perc_size/2);
    % gaussian interaction kernel with local excitation and global inhibition
    field_y_perc_kern=1 *...% strength of gaussian
    gaussNorm(-field_y_perc_half_size:field_y_perc_half_size,...
    0,... % mean value
    5);   % width of gaussian         
      
    % define 2D perceptual field
    field_perc = zeros(field_y_perc_size, field_x_perc_size) + h_d;
    
    % initialize direction to target
    psi_tar = 0;
    % auxilary variables
    psi_tar_dot_aux = 0;
    psi_tar_aux = 0;
    
    %----- initialize state variables ---------------------------------
    % robot state
    robot_pos = [0;0]; % mm (should be integer for the simulator in : 0<x&y<600)
    robot_orient = 0; % rad
    robot_velocity = pi/2; % rad/s
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
    
    
    %----- set parameters of the dynamics -----------------------------
    % step time
    delta_t = 0.02;
    % time constant
    tau_t = 0.06;%0.067;
    
    encoders_old = [];
    encoders_new = [];
   
    ROBOT_WHEEL_RADUIS=21; % mm
    
    % set target state
    target_pos = [300;550];
    
    
    delay = 3000;
    backUntilTime =1; % tells whether bubbleRob is in forward or backward mode
    leftMotorVel = 0;
    rightMotorVel = 0;
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
 	clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
	if (clientID>-1)
       disp('Connected to remote API server');   
       % get objects handles
       [errorCodeleftH,leftMotorHandle]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_oneshot_wait);
       [errorCoderightH,rightMotorHandle]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_oneshot_wait);
       [errorCodesensingNoseH,sensingNoseHandle]=vrep.simxGetObjectHandle(clientID,'remoteApiControlledBubbleRobSensingNose',vrep.simx_opmode_oneshot_wait);
       [errorCodevisionSensorH,visionSensorHandle]=vrep.simxGetObjectHandle(clientID,'ePuck_camera',vrep.simx_opmode_oneshot_wait);
    
       % read initial value of encoders
    
       % get wheels position (wait until real values are recived)
       errorCodeLeftHandle=1;
       errorCodeRightHandle=1;
    
       while ((errorCodeLeftHandle==1)  || (errorCodeRightHandle==1))
             [errorCodeLeftHandle, positionLeftMotor]=vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_continuous);
             [errorCodeRightHandle, positionRightMotor]=vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_continuous);     
             encoders_old(1) =positionLeftMotor*ROBOT_WHEEL_RADUIS; % mm     
             encoders_old(2) =positionRightMotor*ROBOT_WHEEL_RADUIS; % mm          
       end

       % where the program should be. there sould be a loop    
        while (vrep.simxGetConnectionId(clientID)~=-1)  % run the program until simulation will be stopped         
              %----- robot feedback ---------------------------------------------  
              % get wheels position
              [errorCodeLeftHandle, positionLeftMotor]=vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_buffer);
              [errorCodeRightHandle, positionRightMotor]=vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_buffer);                
              
              if ((errorCodeRightHandle==0) && (errorCodeLeftHandle==0))
                 encoders_new(1) =positionLeftMotor*ROBOT_WHEEL_RADUIS; % mm     
                 encoders_new(2) =positionRightMotor*ROBOT_WHEEL_RADUIS; % mm          
              end

              % get delta displacements feedback  (position and orientation in the 
              % allocentric (world) coordinates frame) 
              [encoders_old, delta_pos, delta_phi] = getDeltaPosAllo(encoders_old, encoders_new, phi);

              
              
              % current robot orinetation
              phi = phi + delta_phi; % rad

              % normalize the orientation in the range -pi...pi (to be checked !)
              %phi = normalizeAngle(phi);
       
              % current robot position
              pos = pos + delta_pos'; % mm

              % update robot state (may be for future use)
              robot_pos = pos; % mm
              robot_orient = phi; % rad


            
              % get image information from vision sensor
              %[errorCodevisionSensor,resolution,input_img] = vrep.simxGetVisionSensorImage2(clientID,visionSensorHandle,0,vrep.simx_opmode_continuous);    

              %if (errorCodevisionSensor==0)


%                  %----------------- perceptual field for target aquisition --------
% 
%                  % the nonlinear sigmoidal function output   
%                  field_perc_sig=sigmoid(field_perc,8*beta_d,x0_d);
% 
%                  % calculate inhibition
%                  totalOutput_u = sum(sum(field_perc_sig));
% 
%                  % padding : adding zeros at the end of the output x vector 
%                  field_y_perc_sig_padded = padarray(field_perc_sig, [field_y_perc_half_size, 0], 'circular');
% 
%                  % convolution along y dimention
%                  conv_exc = conv2(field_y_perc_kern, 1, field_y_perc_sig_padded, 'valid');
% 
%                  % padding : adding zeros at the end of the output y vector 
%                  field_x_perc_sig_padded = padarray(conv_exc, [0, field_x_perc_half_size], 'circular'); 
% 
%                  % convolution along y dimention
%                  conv_exc = conv2(1, field_x_perc_kern, field_x_perc_sig_padded, 'valid');
% 
%                  % get the chosen preprocessed color information from the camera 
%                  color_input = getCameraInput(field_x_perc_size, field_y_perc_size, 'red',input_img);
% 
%                  % update field activity 
%                  field_perc = field_perc + (delta_t/tau_t) * (-field_perc + h_d + c_in*color_input + c_exc*conv_exc - g_inh*totalOutput_u )+... % % strength of perception
%                  c_qd * q_d * randn;
% 
%                  % compute peack position (psi_tar)
%                  % inflection point = 1 to handle no abject detection
%                  field_x_perc_sigm = sigmoid(sum(field_perc_sig),8*beta_d,1); 
%                  psi_tar_dot_aux = (-sum(field_x_perc_sigm))*psi_tar_aux+sum((metric_x_step*field_x_perc_nodes).*field_x_perc_sigm);
%                  psi_tar_aux=psi_tar_aux+2*psi_tar_dot_aux*(delta_t);
%                  psi_tar=psi_tar_aux+metric_x_init;

                 % compute the direction and distance to target
                 %[psi_tar, d_tar] = getPsi(robot_pos, target_pos);
                 
                 psi_tar=p/2;
             
                 % compute rate of change of attractive forcelet   
                 delta_phi_tar = getDeltaPhiTarDynamics(psi_tar, phi, lambda_tar, delta_t, tau_t);
                 % compute rate of change of repulsive forcelet   
                 %[delta_phi_obs dist_obs] = getDeltaPhiObsDynamics(phi, beta1_obs,beta2_obs, delta_t, tau_t);

                 % compute heading direction overall rate of change 
                 delta_phi = delta_phi_tar% + delta_phi_obs;


                 % compute right and left wheels speeds
                 [ vr, vl ] = getRotationVelocity(delta_phi, delta_t);
%                  vr = vr + robot_velocity;
%                  vl = vl + robot_velocity;
                 % send commands to the robot
                 if (errorCodeleftH==vrep.simx_error_noerror)
                    vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,vl,vrep.simx_opmode_oneshot);			
                 end
                 if (errorCoderightH==vrep.simx_error_noerror)
                    vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,vr,vrep.simx_opmode_oneshot);
                 end
             
                 



              

                 
                 
                 
              %end
                                          
           end                            
	
		vrep.simxFinish(clientID);
	else
		disp('Failed connecting to remote API server');
	end
	vrep.delete(); % explicitely call the destructor!
	disp('Program ended');
end
 