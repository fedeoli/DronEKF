%% params definition
% define simulation time
params.time=0:1e-3:2;

% init angles (euler ZYX)
eul_0=transpose([0; 0; 0]);

% convert to quaternion
q_start=eul2quat(eul_0);

% init state
params.s0=[3; 3; -3; 0; 0; 0; q_start(1); q_start(2); q_start(3); q_start(4); 0; 0; 0];


% define input
params.tau_story = [0; 0; 0].*ones(3,length(params.time));

% mass
params.m = 0.5;

% inertia matrix
params.I=[0.0081 0 0; ...
          0 0.0081 0;...
          0 0 0.0162];
      
%lunghezza braccio dal centro di massa
params.l=0.2;  

%coefficiente di spinta
params.b=1.46e-5;   

% currently BOOH
params.k=0.026;

% gravity
params.g=0*9.81; 

%Drag factor
params.d=0.0443;       

%parametri motori
params.tau_mot=0.1;    

params.omega0=323;
params.omegaThrust=1*[200; -200; 200; -200];

% params.omegaThrust=[277;... %Caso a) ROLL  (+ ROTORE 1 , -ROTORE3)
%     -267;...
%     257;...
%     -267];
    
% params.omegaThrust=[267;... %Caso b) PITCH (+ ROTORE 2 , -ROTORE4)
%     277;...
%     267;...
%     257];
   
% params.omegaThrust=[367;... %Caso c) YAW (+ ROTORE1/ROTORE3 , - ROTORE2/ROTORE4)
%     267;...
%     367;...
%     267];

%% simulink structure

    

      
      