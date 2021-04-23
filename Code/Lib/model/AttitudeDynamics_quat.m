function dw = AttitudeDynamics_quat(t,w, params)

    % Initialize integration vectors and parameters
    dw = zeros(size(w));

    % Extract attitude, angular velocity 
    q =  w(1:4);
    omega= w(5:7);

    %%%%%%%%%%%%%%%%%% Attitude Kinematics Equations (Quaternions) %%%%%%%%%%%%%%%%%%
    Om = [0, -omega(1), -omega(2), -omega(3);
    omega(1), 0, omega(3), -omega(2);
    omega(2), -omega(3), 0, omega(1);
    omega(3), omega(2), -omega(1), 0];
    dw(1:4,1) = 0.5*Om*q;
    
    %%%%%%%%%%%%%%%%% Attitude Dynamics Equations %%%%%%%%%%%%%%%%%%
    Ix = params.I(1,1);
    Iy = params.I(2,2);
    Iz = params.I(3,3);
    
    tau = params.tau;
    
    pdot = ((Iy-Iz)/Ix)*omega(3)*omega(2) + tau(1)/Ix;
    qdot = ((Iz-Ix)/Iy)*omega(1)*omega(3) + tau(2)/Iy;
    rdot = ((Ix-Iy)/Iz)*omega(2)*omega(1) + tau(3)/Iz;
    dw(5:7)=[pdot; qdot; rdot];

end
