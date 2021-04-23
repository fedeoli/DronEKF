%% frame transformation
function R_out=R(eul)

% rotation along x-axis
R_x = Rx(eul(1));

% rotation along y-axis
R_y = Ry(eul(2));

% rotation along z-axis
R_z = Rz(eul(3));

% final transformation
R_out= R_x*R_y*R_z;



end