%% model integration - synthetic model

% init section
close all
clear 
clc

% simulation time
params.T0 = 0;
params.Tend = 0.5;
params.Ts = 1e-2;
params.time = params.T0:params.Ts:params.Tend;
params.Niter = length(params.time);

% define model
params.att = 'eul';
init_uav;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     INTEGRATION LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start integration
disp('STARTING INTEGRATION PROCEDURE')

for i = 2:params.Niter
    
    % define tspan
    tspan = [params.time(i-1),params.time(i)];
    
    % get input value
    params.tau = params.tau_story(:,i);
    
    %%%%%% integration - attitude %%%%%%
    temp = ode45(@(t,w)AttitudeDynamics_eul(t,w,params),tspan,angle_start(:,i-1));  
    angle_start(:,i) = temp.y(:,end);

    % angoli eulero
    params.eul_story(:,i) = angle_start(1:3,i);
    params.eul = params.eul_story(:,i);        

    % get velocity
    params.omega_story(:,i) = angle_start(4:6,i);
    params.omega = params.omega_story(:,i);

    %%%%%% integration - position %%%%%%
    temp = ode45(@(t,s)PositionDynamics_v2(t,s,params),tspan,pos_start(:,i-1));  
    pos_start(:,i) = temp.y(:,end);

    % get position and velocity
    params.ned_story(:,i) = pos_start(1:3,i);
    params.vel_story(:,i) = pos_start(4:6,i);

end

%%%% plot section %%%%
% andamento della posizione
figure
hold on
grid on
plot3(params.ned_story(1,:),params.ned_story(2,:),params.ned_story(3,:))
plot3(params.ned_story(1,1),params.ned_story(2,1),params.ned_story(3,1),'ro')
plot3(params.ned_story(1,end),params.ned_story(2,end),params.ned_story(3,end),'r+')
xlabel('x')
ylabel('y')
zlabel('z')
