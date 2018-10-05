function calibrateEpuck()

    % using the prototype file (remoteApiProto.m)
    vrep=remApi('remoteApi'); 
    
    % just in case, close all opened connections
    vrep.simxFinish(-1); 
    
    % starts a communication thread with the server (i.e. V-REP)
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
  
    % create a clean up object
    cleanupObj = onCleanup(@()cleanMeUp(clientID,vrep));
    
    delta_t = 0.02;
    time=0;

    if (clientID>-1)
        
        disp('Connected to remote API server');   
        
        % specify the simulation timestep
         vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, ...
             delta_t,  vrep.simx_opmode_oneshot);
        
        % retrieve robot handle 
        errc = zeros(1,4);
        [errc(1),rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
        while (errc(1)~=vrep.simx_error_noerror)  
            [errc(1),rh]=vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
        end

        % get robot position
        [errc(2), rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_blocking);
        rob_pos= rob_pos_vrep(1:2); % m
        while (errc(2)~=vrep.simx_error_noerror)  
            [errc(2), rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_blocking);
            rob_pos= rob_pos_vrep(1:2); % m
        end
        
        % retrieve left joint handle
        jh=zeros(1, 2);
        [errc(3),jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);
        while (errc(3)~=vrep.simx_error_noerror)  
            [errc(3),jh(1)]=vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);
        end

        % get right joint handle
        [errc(4),jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
        while (errc(4)~=vrep.simx_error_noerror)  
            [errc(4),jh(2)]=vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
        end 
    
    
        % set robot initial target velocity to 0
        vrep.simxSetJointTargetVelocity(clientID,jh(1),0,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,jh(2),0,vrep.simx_opmode_oneshot);
        
        % set robot initial position
        vrep.simxSetObjectPosition(clientID,rh,-1,[0 0 0.01915], vrep.simx_opmode_blocking);

        % enable the synchronous mode    
        vrep.simxSynchronous(clientID,true);
        
        % start vrep simulation
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        
        while (vrep.simxGetConnectionId(clientID)>-1)     
        

        
        % get robot position
        [errc(2), rob_pos_vrep]=vrep.simxGetObjectPosition(clientID,rh,-1,vrep.simx_opmode_blocking);
        rob_pos= rob_pos_vrep(1:2); % m   
            
            rob_pos(1);
            
         % send commands to the robot
         vrep.simxPauseCommunication(clientID,1);
         vrep.simxSetJointTargetVelocity(clientID,jh(1),2*pi,vrep.simx_opmode_streaming);			
         vrep.simxSetJointTargetVelocity(clientID,jh(2),2*pi,vrep.simx_opmode_streaming);
         vrep.simxPauseCommunication(clientID,0);
         
         time =time + delta_t;
         

         
         % move simulation ahead one time step
         vrep.simxSynchronousTrigger(clientID);
         vrep.simxGetPingTime(clientID);
        
        if (time>4.98)
            break;
        end
         
        end
    turn=rob_pos(1)/(5);
    wheel_rad=turn/(2*pi)
        else
		disp('Failed connecting to remote API server');
    end
end