function ds = PositionDynamics_v1(t,s, params)

%inizializzazione
ds=zeros(size(s));
%position, linear velocity
ned=s(1:3);
velocity=s(4:6);

xdot=velocity(3)*(sin(params.eul(1))*sin(params.eul(3))+cos(params.eul(1))*cos(params.eul(3))*sin(params.eul(2)))-velocity(2)*(cos(params.eul(1))*sin(params.eul(3))-cos(params.eul(3))*sin(params.eul(1))*sin(params.eul(2)))+velocity(1)*(cos(params.eul(3))*cos(params.eul(2)));
ydot=velocity(2)*(cos(params.eul(1))*cos(params.eul(3))+sin(params.eul(1))*sin(params.eul(3))*sin(params.eul(2)))-velocity(3)*(cos(params.eul(3))*sin(params.eul(1))-cos(params.eul(1))*sin(params.eul(3))*sin(params.eul(2)))+velocity(1)*(cos(params.eul(2))*sin(params.eul(3)));
zdot=velocity(3)*(cos(params.eul(1))*cos(params.eul(2)))-velocity(1)*sin(params.eul(2))+velocity(2)*(cos(params.eul(2))*sin(params.eul(1)));

udot=params.omega(3)*velocity(2)-params.omega(2)*velocity(3)-params.g*sin(params.eul(2));
vdot=params.omega(1)*velocity(3)-params.omega(3)*velocity(1)+params.g*sin(params.eul(1))*cos(params.eul(2));
wdot=params.omega(2)*velocity(1)-params.omega(1)*velocity(2)+params.g*cos(params.eul(2))*cos(params.eul(1));
ds=[xdot;ydot;zdot;udot;vdot;wdot];
end

