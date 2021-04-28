function ds = PositionDynamics_v2(t,s, params)

%inizializzazione
ds=zeros(size(s));
%position, linear velocity
ned=s(1:3);
vel=s(4:6);


%%%%%%%%%%%%%%Position Kinematics Equation (Eulero)%%%%%%%%%%%
R_out=R(params.eul);
ds(1:3)=R_out*vel; %xdot ydot zdot


udot=params.omega(3)*vel(2)-params.omega(2)*vel(3)-params.g*sin(params.eul(2));
vdot=params.omega(1)*vel(3)-params.omega(3)*vel(1)-params.g*cos(params.eul(2))*sin(params.eul(1));
wdot=params.omega(2)*vel(1)-params.omega(1)*vel(2)+params.g*cos(params.eul(1))*cos(params.eul(2))-(params.b*(params.omegaThrust(1)+params.omegaThrust(2)+params.omegaThrust(3)+params.omegaThrust(4)))/params.m;%-T/params.m;
ds(4:6)=[udot;vdot;wdot];
end

