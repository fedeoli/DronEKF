function ds = Dynamics_v1(s, params)



% Initialize integration vectors and parameters

ds = zeros(size(s));

%s= [x y z q0 q1 q2 q3 u v w p q r]

% Extract attitude, angular velocity 
q =  s(4:7);
omega= s(11:13);

%position, linear velocity
ned=s(1:3);
vel=s(8:10);

%quat2eul
eul=quat2eul(q);


%%%%%%%%%%%%%%%%%% Attitude Kinematics Equations (Quaternions) %%%%%%%%%%%%%%%%%%
Om = [0, -omega(1), -omega(2), -omega(3);
    omega(1), 0, omega(3), -omega(2);
    omega(2), -omega(3), 0, omega(1);
    omega(3), omega(2), -omega(1), 0];

ds(4:7,1) = 0.5*Om*q;

omega_thrust = 0.159155*params.omegaThrust;

Mx=params.l*params.b*(omega_thrust(2)^2-omega_thrust(4)^2);
My=params.l*params.b*(omega_thrust(1)^2-omega_thrust(3)^2);
Mz=params.d*(omega_thrust(1)^2-omega_thrust(2)^2+omega_thrust(3)^2-omega_thrust(4)^2);


pdot=(Mx/params.I(1,1))-omega(2)*omega(3)*(params.I(3,3)-params.I(2,2))/params.I(1,1);
qdot=(My/params.I(1,1))-omega(1)*omega(3)*(params.I(1,1)-params.I(3,3))/params.I(2,2);
rdot=(Mz/params.I(3,3));
ds(10:13)=[pdot; qdot; rdot];


%%%%%%%%%%%%%%Position Kinematics Equation (Eulero)%%%%%%%%%%%
R_out=R(eul);
ds(1:3)=R_out*vel; %xdot ydot zdot

omega_thrust = 0.159155*params.omegaThrust;
T = (params.b*(omega_thrust(1)^2 + omega_thrust(2)^2 + omega_thrust(3)^2 + omega_thrust(4)^2));


 udot=params.omega(3)*vel(2)-params.omega(2)*vel(3)-params.g*sin(eul(2));
 vdot=params.omega(1)*vel(3)-params.omega(3)*vel(1)-params.g*cos(eul(2))*sin(eul(1));
 wdot=params.omega(2)*vel(1)-params.omega(1)*vel(2)+params.g*cos(eul(1))*cos(eul(2))-T/params.m;

% udot=params.omega(3)*vel(2)-params.omega(2)*vel(3)-params.g*sin(params.eul(2));
% vdot=params.omega(1)*vel(3)-params.omega(3)*vel(1)-params.g*cos(params.eul(2))*sin(params.eul(1));
% wdot=params.omega(2)*vel(1)-params.omega(1)*vel(2)+params.g*cos(params.eul(1))*cos(params.eul(2))-T/params.m;
ds(8:10)=[udot;vdot;wdot];

end