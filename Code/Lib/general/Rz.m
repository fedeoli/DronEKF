%% x-axis rotation matrix
function Rz = Rz(eul)

    % rotation along z-axis
    Rz =    [cos(eul) -sin(eul) 0;...
             sin(eul) cos(eul) 0;...
             0 0 1];
end
