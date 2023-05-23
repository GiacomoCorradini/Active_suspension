%% Setting

clc
close all
clear all

%% Setting Params -> E-AGLE TRT car (Fenice)

ms  = 234;   % Sprung Mass (kg)
mus = 43;    % Unsprung Mass (kg)
ks  = 26000; % Suspension Stiffness (N/m) 
bs  = 1544;  % Suspension Inherent Damping coefficient (sec/m)
kus = 10^5;  % Wheel stiffness (N/m)
bus = 40000; % Wheel Inhenrent Damping coefficient (sec/m)

% zs  -> sprung mass displacement
% zus -> unsprung mass displacement
% zs  -> sprung mass velocity
% zus -> unsprung mass velocity
% zr  -> road disturbances

%% Road profile

T_f = 5;
sim_time = 0.01:0.04:T_f;
road_surface = zeros(2,length(sim_time));

for i = 1:length(sim_time)
    road_surface(1,i) = sim_time(i);  
    if ((sim_time(i)>=0.5) && (sim_time(i)<=0.75))
        road_surface(2,i) = (0.055*(1-cos(8*pi*sim_time(i))));
    elseif (sim_time(i)>=3) && (sim_time(i)<=3.25)
        road_surface(2,i) = (0.025*(1-cos(8*pi*sim_time(i))));
      else
        road_surface(2,i) = 0;  
    end
end

road_surface = timeseries(road_surface(2,:),sim_time);

% figure('Name','Road profile')
% plot(road_surface)


%% Suspension dynamics

% x_dot = A*x + B*u
%   y   = C*x + D*u

% x = [x1,x2,x3,x4] -> [zs-zus,zs_dot,zus-zr,zus_dot]
% u = [xr, ft]
% y = [y1,y2,y3,y4,y5] -> [zs-zus,zs_dot,zus-zr,zus_dot,zs_dotdot]

A = [   0       1        0          -1; 
     -ks/ms  -bs/ms      0         bs/ms ;
        0       0        0           1; 
      ks/mus  bs/mus -kus/mus -(bs+bus)/mus];

B = [  0       0; 
       0      1/ms; 
       -1      0; 
     bus/mus -1/mus];

C = [        eye(4)
     -ks/ms -bs/ms   0 bs/ms];

D = [zeros(4,2)
     0 1/ms];

%% Controllability

% the controllability matrix must be full ranck
rank(ctrb(A,B))

%% LQR Control law

% Must be tuned
Q = diag([1760*10^6, 11.6*10^6, 1, 1]);
R = 0.01;

% feedback gain vector
K = lqr( A, B(:,2), Q, R )

%% Start Simulation
%------------------------------------------------------------------------------
fprintf( 'Starting Simulation\n' )

tic; % start measuring time

% Run the simulation
model_sim = sim('Active_Suspension_model.slx');

elapsed_time_simulation = toc; % stop measuring time

fprintf( 'Simulation completed\n' )
fprintf( 'The total simulation time was %.2f seconds\n', elapsed_time_simulation)

%% 

qcar = ss(A,B,C,D);
qcar.StateName = {'zs-zus (m)';'zs_dot (m/s)';'zus-zr (m)';'zus_dot (m/s)'};
qcar.InputName = {'zr';'ft'};
qcar.OutputName = {'zs-zus (m)';'zs_dot (m/s)';'zus-zr (m)';'zus_dot (m/s)';'zs_dotdot (m/s^2)'};

tzero(qcar({'xb','ab'},'ft'))

%% Extreact results

state = model_sim.state;

figure('Name','State signals'); hold on;
plot(state.signal1)
plot(state.signal2)
plot(state.signal3)
plot(state.signal4)
plot(state.signal5)
legend({'zs-zus','zs_dot','zus-zr','zus_dot','zs_dotdot'})