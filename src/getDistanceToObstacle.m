function [errc, detectionState, dist_obs] = getDistanceToObstacle(clientID,vrep,errc,psh)
% compute distances to obstacles through the proximity sensors output

% get sensed distances to obstacles from the 8 infra-red sensors (light sensors)
% sensor range can be found in the detection volume properties of the proximity
% sensor in vrep simulator
senc_range=0.04; % m

dist_obs = senc_range*ones(1, 8);
detectionState = zeros(1, 8);
for i = 22:29
    [errc(i), detectionState(i-21), detectedPoint]=vrep.simxReadProximitySensor(clientID, psh(i-21), vrep.simx_opmode_streaming);
    if detectionState(i-21)
        dist_obs(i-21) = min(sqrt(detectedPoint(1)^2+detectedPoint(2)^2+detectedPoint(3)^2),senc_range); % m
    end
end
end
