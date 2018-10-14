function [dist_obs] = getDistanceToObstacle(clientID,vrep,psh)
% compute distances to obstacles through the proximity sensors output

% get sensed distances to obstacles from the 8 infra-red sensors (light sensors)
% sensor range can be found in the detection volume properties of the proximity
% sensor in vrep simulator
senc_range=0.04; % m

dist_obs = senc_range*ones(1, 8); % m
detectionState = zeros(1, 8);
for i = 1:size(psh,2)
    [err_code, detectionState(i), detectedPoint]=vrep.simxReadProximitySensor(clientID, psh(i), vrep.simx_opmode_blocking);
    if detectionState(i)
        dist_obs(i) = min(sqrt(detectedPoint(1)^2+detectedPoint(2)^2+detectedPoint(3)^2),senc_range); % m
    end
end
end
