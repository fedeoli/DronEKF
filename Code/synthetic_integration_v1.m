%% model integration - synthetic model

close all
clear all

% define simulation time
params.time=0:1e-3:2;

% init angles (euler ZYX)
eul_0=transpose([0; 0; 0]);
% convert to quaternion
q_start=eul2quat(eul_0);

% init state
params.w0=[q_start(1); q_start(2); q_start(3); q_start(4); 0; 0; 0]; %7x1 [q0 q1 q2 q3 p q r] quaternioni e velocità angolari (pqr sistema di riferimento ABC)
params.s0=[3; 3; 3; 0;0;0]; %6x1 [x y z u v w] xyz sistema di riferimento NED, uvw sistema di riferimento ABC

% define input
params.tau_story = [0; 0; 0].*ones(3,length(params.time));

%define model
init_uav;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     INTEGRATION LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init procedure
%%% init state %%%

pos_start =params.s0; %POSITION
params.ned_story=pos_start(1:3);

angle_start = params.w0; %ATTITUDE
params.eul_story=quat2eul(angle_start(1:4)')';
params.omega_story=angle_start(5:7);


% start integration
disp('STARTING INTEGRATION PROCEDURE')

for i = 2:length(params.time)
    params.tau=params.tau_story(:,i);
    tspan=[params.time(i-1),params.time(i)];

    % integration     
    temp =ode45(@(t,w)AttitudeDynamics_v1(t,w,params),tspan,angle_start(:,i-1));  
    angle_start(:,i) = temp.y(:,end);

    % quaternioni to eulero
    params.eul_story(:,i) = quat2eul(angle_start(1:4,i)');
    params.eul = params.eul_story(:,i);        

    params.omega_story(:,i)=angle_start(5:7,i);
    params.omega= params.omega_story(:,i);

    %%% integrate position 
    temp =ode45(@(t,s)PositionDynamics_v2(t,s,params),tspan,pos_start(:,i-1));  
    pos_start(:,i) = temp.y(:,end);

    params.ned_story(:,i)=pos_start(1:3,i);
    params.vel_story(:,i)=pos_start(4:6,i);

end

%plot2 XYZ   
figure
hold on
grid on
plot(params.time,params.ned_story(1,:))
plot(params.time,params.ned_story(2,:))
plot(params.time,params.ned_story(3,:))
lgd=legend('X','Y','Z');
title(lgd,'Coordinate NED')  


%plot2 velocità lineari UVW
figure
hold on
grid on
plot(params.time,params.vel_story(1,:))
plot(params.time,params.vel_story(2,:))
plot(params.time,params.vel_story(3,:))
lgd=legend('U','V','W');
title(lgd,'Velocità lineari su XYZ')    




%plot2 angoli di eulero
figure 
hold on
grid on
plot(params.time,params.eul_story(1,:))
plot(params.time,params.eul_story(2,:))
plot(params.time,params.eul_story(3,:))
lgd=legend('phi','theta','psi');
title(lgd,'Angoli di Eulero')



%plot2 velocità angolari
figure
hold on
grid on
plot(params.time,params.omega_story(1,:))
plot(params.time,params.omega_story(2,:))
plot(params.time,params.omega_story(3,:))
lgd=legend('p','q','r');
title(lgd,'Velocità angolari')

%plot3 andamento della posizione
figure
hold on
grid on
plot3(params.ned_story(1,:),params.ned_story(2,:),params.ned_story(3,:))
plot3(params.ned_story(1,1),params.ned_story(2,1),params.ned_story(3,1),'ro')
plot3(params.ned_story(1,end),params.ned_story(2,end),params.ned_story(3,end),'r+')
legend('Posizione')










