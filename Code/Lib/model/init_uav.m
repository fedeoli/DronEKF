%% define drone physics
% mass
params.m=0.85;

% inertia
params.I=[0.0081 0 0; ...
          0 0.0081 0;...
          0 0 0.0162];

%lunghezza braccio dal centro di massa
params.l=0.2;

%coefficiente di spinta
params.b=1.46e-5; 

% TBD
params.k=0.026;

% gravitational acceleration
params.g=9.81;

%parametri motori
params.tau_mot=0.1;
params.omega0=323;

%%% define simulation init vals %%%
%%%%%%%%%% EULER ANGLES %%%%%%%%%%%
if strcmp(params.att,'eul')
    % initial condition - attitude
    eul_0 = [0; 0; pi/4];
    w0 = [0; 0; 0];
    % 7x1 [q0 q1 q2 q3 p q r] 
    params.w0 = [eul_0; w0]; 

    % initial condition - position
    ned_0 = [4; 3; 3];
    v0 = [0; 0; 0];
    % 6x1 [x y z u v w] 
    params.s0=[ned_0; v0];

    % define input
    params.tau_story = [0; 0; 0].*ones(3,params.Niter);
    
    %%% init state - position %%%
    pos_start = params.s0; 
    params.ned_story=pos_start(1:3);
    params.vel_story=pos_start(4:6);

    %%% init state - position %%%
    angle_start = params.w0; 
    params.eul_story = angle_start(1:3);
    params.omega_story = angle_start(4:6);
    
%%%%%%%%%% QUATERNIONS %%%%%%%%%%%
elseif strcmp(params.att,'quat')
   % initial condition - attitude
    eul_0 = [0; 0; 0];
    q_start = eul2quat(eul_0')';
    w0 = [0; 0; 0];
    % 7x1 [q0 q1 q2 q3 p q r] 
    params.w0 = [q_start; w0]; 

    % initial condition - position
    ned_0 = [4; 3; 3];
    v0 = [0; 0; 0];
    % 6x1 [x y z u v w] 
    params.s0=[ned_0; v0];

    % define input
    params.tau_story = [0; 0; 0].*ones(3,length(params.time));
    
    %%% init state - position %%%
    pos_start = params.s0; 
    params.ned_story=pos_start(1:3);
    params.vel_story=pos_start(4:6);

    %%% init state - position %%%
    angle_start = params.w0; 
    params.quat_story = angle_start(1:4);
    params.eul_story=quat2eul(angle_start(1:4)')';
    params.omega_story=angle_start(5:7);
else
    disp('error')
    return
end
      
      