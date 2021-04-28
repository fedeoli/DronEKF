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


% pdot=((params.I(2,2)-params.I(3,3))/params.I(1,1))*omega(3)*omega(2)+params.tau(1)/params.I(1,1);
% qdot=((params.I(3,3)-params.I(1,1))/params.I(2,2))*omega(1)*omega(3)+params.tau(2)/params.I(2,2);
% rdot=((params.I(1,1)-params.I(2,2))/params.I(3,3))*omega(2)*omega(1)+params.tau(3)/params.I(3,3);
pdot=((params.l*params.b)/params.I(1,1))*(params.omegaThrust(2)^2-params.omegaThrust(2)^2)-omega(2)*omega(3)*(params.I(3,3)-params.I(2,2))/params.I(1,1);
qdot=((params.l*params.b)/params.I(1,1))*(params.omegaThrust(1)^2-params.omegaThrust(3)^2)-omega(1)*omega(3)*(params.I(1,1)-params.I(3,3))/params.I(2,2);
rdot=(params.d/params.I(3,3))*(params.omegaThrust(1)^2-params.omegaThrust(2)^2+params.omegaThrust(3)^2-params.omegaThrust(4)^2);
dw(5:7)=[pdot; qdot; rdot];

end
