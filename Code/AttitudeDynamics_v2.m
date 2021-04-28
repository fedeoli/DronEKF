function dw = AttitudeDynamics_v2(t,w, params)



% Initialize integration vectors and parameters

dw = zeros(size(w));


% attitude, angular velocity 
eul =  w(1:3);%phi theta psi
omega= w(4:6); %p q r


%%%%%%%%%%%%%%%%%% Attitude Kinematics Equations (Eulero) %%%%%%%%%%%%%%%%%%
T_out=T(eul);

dw(1:3)=T_out*omega; %phidot thetadot psidot


pdot=((params.I(2,2)-params.I(3,3))/params.I(1,1))*omega(3)*omega(2)+params.tau(1)/params.I(1,1);
qdot=((params.I(3,3)-params.I(1,1))/params.I(2,2))*omega(1)*omega(3)+params.tau(2)/params.I(2,2);
rdot=((params.I(1,1)-params.I(2,2))/params.I(3,3))*omega(2)*omega(1)+params.tau(3)/params.I(3,3);

dw(4:6)=[pdot; qdot; rdot]; %pdot qdot rdot

end
