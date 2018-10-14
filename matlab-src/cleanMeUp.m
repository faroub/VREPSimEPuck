function cleanMeUp(clientID,vrep)
    % make sure that the last command sent out had time to arrive
    vrep.simxGetPingTime(clientID);
    % stop vrep simulation from Matlab script
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
    % close the line if still open
    vrep.simxFinish(clientID); 
     % explicitely call the destructor!
    vrep.delete();
    % clear global variables 
    clear global ROBOT_WHEEL_RADUIS ROBOT_DISTANCE_BETWEEN_WHEELS ROBOT_PROXIMITY_SENSORS_DIRECTIONS M2MM MM2M RAD2DEG DEG2RAD
    disp('Program ended');
end