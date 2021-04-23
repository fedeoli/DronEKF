%% x-axis rotation matrix
function Rx = Rx(eul)

    % rotation along x-axis
    Rx =    [1 0 0;...
             0 cos(eul) -sin(eul);...
             0 sin(eul) cos(eul)];
end
