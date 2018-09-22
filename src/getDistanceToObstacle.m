function [errc, detectionState, dist_obs] = getDistanceToObstacle(clientID,vrep,errc,psh)
% compute distances to obstacles through the proximity sensors output
global M2MM
% get sensed distances to obstacles from the 8 infra-red sensors (light sensors)
dist_obs = zeros(1, 8);
detectionState = zeros(1, 8);
for i = 18:25
    [errc(i), detectionState(i-17), detectedPoint]=vrep.simxReadProximitySensor(clientID, psh(i-17), vrep.simx_opmode_streaming);
    dist_obs(i-17) = sqrt(detectedPoint(1)^2+detectedPoint(2)^2+detectedPoint(3)^2)*M2MM; % mm
end



%[errc,boolean detectionState,array detectedPoint,number detectedObjectHandle,array detectedSurfaceNormalVector]=simxReadProximitySensor(number clientID,number sensorHandle,number operationMode)

% sensed distances to obstacles from the 8 infra-red sensors (light sensors)



% sensors = kProximity( ROBOT_HANDLE );      
% % make sure that the vector pocceses 8 elements
% if size(sensors,1) ~= 8
%     sensors = zeros(8,1);
% end
% % readings should not be equal to zero (only a reading between max and min is accepted)
% % max = ROBOT_PROXIMITY_SENSOR_MAX and min = 1;
% sensors=max(min(sensors,ROBOT_PROXIMITY_SENSOR_MAX),1); 
% % normalize sensors reading and use "-log(x)"  as a fitting function to
% % obtain distances in mm from robot center
% dist_obs = 15-10*log( sensors ./ ROBOT_PROXIMITY_SENSOR_MAX); % mm


end
