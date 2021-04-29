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
%     267;...
%     257;...
%     267];
    
% params.omegaThrust=[267;... %Caso b) PITCH (+ ROTORE 2 , -ROTORE4)
%     277;...
%     267;...
%     257];
   
% params.omegaThrust=[367;... %Caso c) YAW (+ ROTORE1/ROTORE3 , - ROTORE2/ROTORE4)
%     267;...
%     367;...
%     267];
    
    

      
      