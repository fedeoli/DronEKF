%% model integration - synthetic model

close all
clear all

    params.time=0:0.01:0.5;
    
    eul_0=[0; pi/2; 0]';
    
    params.w0=[eul_0(1); eul_0(2); eul_0(3); 0; 0; 0]; %6x1 [phi theta psi p q r] angoli di eulero e velocità angolari (pqr sistema di riferimento ABC)
    params.s0=[4; 3; 3; 0;0;0]; %6x1 [x y z u v w] xyz sistema di riferimento NED, uvw sistema di riferimento ABC
     
    
   %define input
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
    params.eul_story=angle_start(1:3);
    params.omega_story=angle_start(4:6);
    
    
    % start integration
    disp('STARTING INTEGRATION PROCEDURE')
    
    for i = 2:length(params.time)
        params.tau=params.tau_story(:,i);
        tspan=[params.time(i-1),params.time(i)];
        
        % integration     
        temp =ode45(@(t,w)AttitudeDynamics_v2(t,w,params),tspan,angle_start(:,i-1));  
        angle_start(:,i) = temp.y(:,end);

       
        params.eul_story(:,i) = angle_start(1:3,i);
        params.eul = params.eul_story(:,i);
        
        params.omega_story(:,i)=angle_start(4:6,i);
        params.omega= params.omega_story(:,i);
        
        %%% integrate position 
        temp =ode45(@(t,s)PositionDynamics_v2(t,s,params),tspan,pos_start(:,i-1));  
        pos_start(:,i) = temp.y(:,end);
        
        params.ned_story(:,i)=pos_start(1:3,i);
        params.vel_story(:,i)=pos_start(4:6,i);
        
    end
    
    
%andamento della posizione
figure
hold on
grid on
plot3(params.ned_story(1,:),params.ned_story(2,:),params.ned_story(3,:))
plot3(params.ned_story(1,1),params.ned_story(2,1),params.ned_story(3,1),'ro')
plot3(params.ned_story(1,end),params.ned_story(2,end),params.ned_story(3,end),'r+')

%velocità angolari
figure
hold on
grid on
plot(params.time,params.omega_story(1,:))
plot(params.time,params.omega_story(2,:))
plot(params.time,params.omega_story(3,:))

%angoli di eulero
figure 
hold on
grid on
plot(params.time,params.eul_story(1,:))
plot(params.time,params.eul_story(2,:))
plot(params.time,params.eul_story(3,:))
grid on

