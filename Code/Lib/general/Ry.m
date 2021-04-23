%% x-axis rotation matrix
function Ry = Ry(eul)

    % rotation along y-axis
    Ry =    [cos(eul) 0 sin(eul);...
             0 1 0;...
             -sin(eul) 0 cos(eul)];
end
