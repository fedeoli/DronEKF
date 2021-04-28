params.m=0.85;
params.I=[0.0081 0 0; ...
          0 0.0081 0;...
          0 0 0.0162];
params.l=0.2;          %lunghezza braccio dal centro di massa
params.b=1.46e-5;      %coefficiente di spinta
params.k=0.026;
params.g=9.81; %9.81;
params.d=0.0443;       %Drag factor

params.tau_mot=0.1;    %parametri motori

params.omega0=323;
params.omegaThrust=[-1000;... %Spinta dei motori
    1000;...
    -1000;...
    1000];

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
    
    

      
      