function dw = AttitudeDynamics_v1(t,w, params)



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

omega_thrust = 0.159155*params.omegaThrust;
Mx=params.l*params.b*(omega_thrust(2)^2-omega_thrust(4)^2);
My=params.l*params.b*(omega_thrust(1)^2-omega_thrust(3)^2);
Mz=params.d*(omega_thrust(1)^2-omega_thrust(2)^2+omega_thrust(3)^2-omega_thrust(4)^2);



pdot=(Mx/params.I(1,1))-omega(2)*omega(3)*(params.I(3,3)-params.I(2,2))/params.I(1,1);
qdot=(My/params.I(1,1))-omega(1)*omega(3)*(params.I(1,1)-params.I(3,3))/params.I(2,2);
rdot=(Mz/params.I(3,3));
dw(5:7)=[pdot; qdot; rdot];

end
