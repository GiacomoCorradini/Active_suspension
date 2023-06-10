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
bus = 0; % Wheel Inhenrent Damping coefficient (sec/m)

% zs  -> sprung mass displacement
% zus -> unsprung mass displacement
% zs  -> sprung mass velocity
% zus -> unsprung mass velocity
% zr  -> road disturbances

%% Road profile

T_f = 10;
sim_time = 0.01:0.04:T_f;
road_surface = zeros(2,length(sim_time));

% more realistic road profile
for i = 1:length(sim_time)
    road_surface(1,i) = sim_time(i);  
    if ((sim_time(i)>0.5) && (sim_time(i)<0.75))
        road_surface(2,i) = (0.5*(1-sin(0.8*pi*sim_time(i))));
    else
        road_surface(2,i) = 0;  
    end
end

road_surface = timeseries(road_surface(2,:),sim_time);

figure('Name','Road profile')
plot(road_surface)
xlabel('Time (s)')
ylabel('zr [m]')
title('Road surface displacement')

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

%% PID Tuning

qcar = ss(A,B,C,D);
qcar.StateName = {'zs-zus (m)';'zs_dot (m/s)';'zus-zr (m)';'zus_dot (m/s)'};
qcar.InputName = {'zr_d';'Fa'};
qcar.OutputName = {'zs-zus';'zs_dot';'zus-zr';'zus_dot';'zs_dotdot'};

pid = pidtune(qcar(1,2),'PID');
Kp = pid.Kp;
Kd = pid.Kd;
Ki = pid.Ki;

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

%% Extreact results

state_passive = model_sim.state.state_passive;
state_LQR = model_sim.state.state_LQR;
state_PID = model_sim.state.state_PID;

sprung_mass_a = model_sim.sprung_mass_acc;

figure('Name','Suspension travel'); hold on;
plot(state_passive.zs_zus)
plot(state_LQR.zs_zus)
plot(state_PID.zs_zus)
legend({'passive','LQR','PID'})
title('Suspension travel')

figure('Name','Sprung mass velocity'); hold on;
plot(state_passive.zs_d)
plot(state_LQR.zs_d)
plot(state_PID.zs_d)
legend({'passive','LQR','PID'})
title('Sprung mass velocity')

figure('Name','Tire deflection'); hold on;
plot(state_passive.zus_zr)
plot(state_LQR.zus_zr)
plot(state_PID.zus_zr)
legend({'passive','LQR','PID'})
title('Tire deflection')

figure('Name','Unsprung mass velocity'); hold on;
plot(state_passive.zus_d)
plot(state_LQR.zus_d)
plot(state_PID.zus_d)
legend({'passive','LQR','PID'})
title('Unsprung mass velocity')

figure('Name','Sprung mass acceleration'); hold on;
plot(sprung_mass_a.zs_dd_passive)
plot(sprung_mass_a.zs_dd_LQR)
plot(sprung_mass_a.zs_dd_PID)
legend({'passive','LQR','PID'})
title('Sprung mass acceleration')

%% Evaluation param

sp_passive = state_passive.zs_zus.Data;
sp_LQR = state_LQR.zs_zus.Data;
sp_PID = state_PID.zs_zus.Data;

msd_passive = state_passive.zs_d.Data;
msd_LQR = state_LQR.zs_d.Data;
msd_PID = state_PID.zs_d.Data;

td_passive = state_passive.zus_zr.Data;
td_LQR = state_LQR.zus_zr.Data;
td_PID = state_PID.zus_zr.Data;

musd_passive = state_passive.zus_d.Data;
musd_LQR = state_LQR.zus_d.Data;
musd_PID = state_PID.zus_d.Data;

msdd_passive = sprung_mass_a.zs_dd_passive.Data;
msdd_LQR = sprung_mass_a.zs_dd_LQR.Data;
msdd_PID = sprung_mass_a.zs_dd_PID.Data;



% Suspension travel
sp_LQR_index = (1 - abs(rms(sp_LQR))/abs(rms(sp_passive)))*100
sp_PID_index = (1 - abs(rms(sp_PID))/abs(rms(sp_passive)))*100

% Tire deflection
msd_LQR_index = (1 - abs(rms(msd_LQR))/abs(rms(msd_passive)))*100
msd_PID_index = (1 - abs(rms(msd_PID))/abs(rms(msd_passive)))*100

% Sprung mass velocity
td_LQR_index = (1 - abs(rms(td_LQR))/abs(rms(td_passive)))*100
td_PID_index = (1 - abs(rms(td_PID))/abs(rms(td_passive)))*100

% Unsprung mass velocity
musd_LQR_index = (1 - abs(rms(musd_LQR))/abs(rms(musd_passive)))*100
musd_PID_index = (1 - abs(rms(musd_PID))/abs(rms(musd_passive)))*100

% Sprung mass acceleration
msdd_LQR_index = (1 - abs(rms(msdd_LQR))/abs(rms(msdd_passive)))*100
msdd_PID_index = (1 - abs(rms(msdd_PID))/abs(rms(msdd_passive)))*100

