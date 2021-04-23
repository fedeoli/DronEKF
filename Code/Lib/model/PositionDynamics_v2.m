function ds = PositionDynamics_v2(t,s, params)

    % inizializzazione
    ds=zeros(size(s));
    
    % position, linear velocity
    ned=s(1:3); % useless
    vel=s(4:6);
    
    % euler angles
    eul = params.eul;
    
    % angular velocity
    omega = params.omega;

    %%%%%%%%%%%%%% Position Kinematics Equation (Eulero) %%%%%%%%%%%    
    R_out=R(eul);
    ds(1:3)=R_out*vel; %xdot ydot zdot

    %%%%%%%%%%%%% Position Dynamics Equation %%%%%%%%%%%%%%%%%%
    g = params.g;
    udot = omega(3)*vel(2)-omega(2)*vel(3)-g*sin(eul(2));
    vdot = omega(1)*vel(3)-omega(3)*vel(1)-g*cos(eul(2))*sin(eul(1));
    wdot = omega(2)*vel(1)-omega(1)*vel(2)+g*cos(eul(1))*cos(eul(2));
    
    ds(4:6)=[udot;vdot;wdot];
end

