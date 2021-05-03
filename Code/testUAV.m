%% UAV matlab toolbox
clear 
close all
clc

%% model definition
model = multirotor;
model.Name = 'MasterDrone';
model.Configuration.PDPitch = 0*model.Configuration.PDPitch;
model.Configuration.PDRoll = 0*model.Configuration.PDRoll;
model.Configuration.PThrust = 0*model.Configuration.PThrust;
model.Configuration.PYawRate = 0*model.Configuration.PYawRate;

%% state definition
s = state(model);
% define initial position
s(1:3) = [4;4;3];
% define initial velocity
s(4:6) = [0;0;0];
% define initial quaternion
s(7:10) = [1; 0; 0; 0];
% define initial angular velocity
s(11:13) = [0;0;0];

%% define control definition
u = control(model);

%% define environment
e = environment(model);

%% integrate model
simOut = ode45(@(~,x)derivative(model,x,u,e), [0 3], s);
size(simOut.y)

%% plot section
% position
figure
hold on
grid on
plot(simOut.y(1,:));
plot(simOut.y(2,:));
plot(simOut.y(3,:));
legend('X-position','Y-position','Z-position')
hold off

% attitude
eul = zeros(length(simOut.y),3);
for i=1:length(simOut.y)
   eul(i,:) = quat2eul(transpose(simOut.y(7:10,i)));
end
figure
hold on
grid on
plot(eul(1,:));
plot(eul(2,:));
plot(eul(3,:));
legend('Phi','Theta','Psi')
hold off