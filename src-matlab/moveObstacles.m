function  [obs_positions]=moveObstacles(clientID, vrep, obsh, obs_positions, obs_move_step)
% move randomly the obstacles during simulation 

% distance threshold to obstacle 
d_th=0.15; % m

% coordinates of simulation scene corners
x_max=0.75; % m
x_min=-0.75; % m
y_max=1.25; % m
y_min=-0.25; % m

% reset obstacles movement steps
for i = 1:size(obs_positions,1)
    obs_positions(i,5)=obs_move_step;
end

% check obstacles positions in simulation scene
% check if obstacles reached  borders
for i = 1:size(obs_positions,1)

 if ((obs_positions(i,1)==x_max) || (obs_positions(i,1)==x_min) ...
  || (obs_positions(i,2)==y_max) || (obs_positions(i,2)==y_min))
    
    % change obstacle behavior
    obs_positions(i,4)=randi([1 8],1);
    
    % increase temporary obstacle movement step size
    obs_positions(i,5)=2*obs_move_step; % m
    
 end

end

% check if  collisions are imminent 
for i = 1:size(obs_positions,1)   
    for j= 1:size(obs_positions,1)
        if i~=j
            % distance to  obstacles
            d_obs = sqrt((obs_positions(i,1)-obs_positions(j,1))^2+(obs_positions(i,2)-obs_positions(j,2))^2);
            
            % check if distance less than threshold
            if (d_obs <= d_th)

                % change obstacle behavior
                if (obs_positions(i,4)<=4)
                    obs_positions(i,4)=obs_positions(i,4)+4;   
                else
                    obs_positions(i,4)=obs_positions(i,4)-4;
                end
                
                % increase temporary obstacle movement step size
                obs_positions(i,5)=2*obs_move_step; % m

            end
        end
    end
end


% generate movement behaviors
%    4  3  2
%     \  |  /
% 5 -- o -- 1
%     /  |  \
%    6  7  8
for i = 1:size(obs_positions,1)
    
    obs_move_step=obs_positions(i,5);
    
    switch obs_positions(i,4);
        case 1
            obs_positions(i,1)=min(max(obs_positions(i,1)+obs_move_step, x_min), x_max);
        case 2
            obs_positions(i,1)=min(max(obs_positions(i,1)+obs_move_step, x_min), x_max);
            obs_positions(i,2)=min(max(obs_positions(i,2)+obs_move_step, y_min), y_max);
        case 3
            obs_positions(i,2)=min(max(obs_positions(i,2)+obs_move_step, y_min), y_max);
        case 4
            obs_positions(i,1)=min(max(obs_positions(i,1)-obs_move_step, x_min), x_max);
            obs_positions(i,2)=min(max(obs_positions(i,2)+obs_move_step, y_min), y_max);
        case 5
            obs_positions(i,1)=min(max(obs_positions(i,1)-obs_move_step, x_min), x_max);
        case 6
            obs_positions(i,1)=min(max(obs_positions(i,1)-obs_move_step, x_min), x_max);
            obs_positions(i,2)=min(max(obs_positions(i,2)-obs_move_step, y_min), y_max);
        case 7
            obs_positions(i,2)=min(max(obs_positions(i,2)-obs_move_step, y_min), y_max);
        case 8
            obs_positions(i,1)=min(max(obs_positions(i,1)+obs_move_step, x_min), x_max);
            obs_positions(i,2)=min(max(obs_positions(i,2)-obs_move_step, y_min), y_max);
    end
    
end

% set the obstacles current positions
for i = 1:size(obs_positions,1)
    [err_code]=vrep.simxSetObjectPosition(clientID,obsh(i),-1, obs_positions(i,1:3), vrep.simx_opmode_blocking);
    while (err_code~=vrep.simx_error_noerror)
        [err_code]=vrep.simxSetObjectPosition(clientID,obsh(i),-1, obs_positions(i,1:3), vrep.simx_opmode_blocking);
    end
end

end

