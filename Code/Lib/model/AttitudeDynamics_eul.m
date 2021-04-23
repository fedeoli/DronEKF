function dw = AttitudeDynamics_eul(t,w, params)

    % Initialize integration vectors and parameters
    dw = zeros(size(w));

    % Extract attitude, angular velocity 
    eul =  w(1:3);
    omega= w(4:6);

    %%%%%%%%%%%%%%%%%% Attitude Kinematics Equations (Eulero) %%%%%%%%%%%%%%%%%%
    T_out=T(eul);
    dw(1:3)=T_out*omega; %phidot thetadot psidot

    %%%%%%%%%%%%%%%%% Attitude Dynamics Equations %%%%%%%%%%%%%%%%%%
    Ix = params.I(1,1);
    Iy = params.I(2,2);
    Iz = params.I(3,3);
    
    tau = params.tau;
    
    pdot = ((Iy-Iz)/Ix)*omega(3)*omega(2) + tau(1)/Ix;
    qdot = ((Iz-Ix)/Iy)*omega(1)*omega(3) + tau(2)/Iy;
    rdot = ((Ix-Iy)/Iz)*omega(2)*omega(1) + tau(3)/Iz;
    dw(4:6)=[pdot; qdot; rdot];

end
