function [tar_pos, tar_idx ] = getTargetPosition( tar_positions, tar_pos, d_tar, tar_nbr, tar_idx)
% get the target positions and switch between them during simulation 
% tar_idx=tar_idx;
% tar_pos=tar_pos;
% target threshold
d_th=0.005; % m

% select one of the targets
if (d_tar<= d_th)
    tar_idx=tar_idx+1
    if (tar_idx<=4)
        tar_pos = tar_positions(tar_nbr(tar_idx),:);
    end
    
    
end





end

