%% Setting

clc
close all
clear all

warning off

% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

set(0,'defaultAxesFontSize',22)
set(0,'DefaultLegendFontSize',22)
set(0,'DefaultFigureWindowStyle','docked')
set(0,'DefaultUicontrolFontsize', 14)
set(0, 'DefaultLineLineWidth', 3);
% set(findall(gcf,'type','line'),'linewidth',2)

%% Setting Params -> E-AGLE TRT car (Fenice)

ms  = 234;   % Sprung Mass (kg) (all)
mus = 43;    % Unsprung Mass (kg)

% equivalent spring -> parallel of all springs 
% ks = sum of all ks_i
ks  = 26000*4; % Suspension Stiffness (N/m) 
% equivalent damper -> parallel of all dampers 
% bs = sum of all bs_i
bs  = 1544*4;  % Suspension Inherent Damping coefficient (N sec/m)
% equivalent spring -> parallel of all springs 
% kus = sum of all kus_i
kus = 100000*4;  % Wheel stiffness (N/m)
% equivalent damper -> parallel of all dampers 
% bus = sum of all bus_i
bus = 0*4;     % Wheel Inhenrent Damping coefficient (N sec/m)

% zs  -> sprung mass displacement
% zus -> unsprung mass displacement
% zs  -> sprung mass velocity
% zus -> unsprung mass velocity
% zr  -> road disturbances

%% Road profile

T_f = 5;
sim_time = 0.01:0.04:T_f;
road_surface = zeros(2,length(sim_time));

% choose velocity
% road = 1; % 20 km/h
% road = 2; % 40 km/h
road = 3; % 72 km/h

% bump road profile
for i = 1:length(sim_time)
    road_surface(1,i) = sim_time(i);  
    if ((sim_time(i)>1) && (sim_time(i)<1.09)) && road == 1 % 20 km/h
        road_surface(2,i) = (0.04*(1-cos(7*pi*sim_time(i))));
    elseif ((sim_time(i)>1) && (sim_time(i)<1.045)) && road == 2 % 40 km/h
        road_surface(2,i) = (0.04*(1-cos(7*pi*sim_time(i))));
    elseif ((sim_time(i)>1) && (sim_time(i)<1.025)) && road == 3 % 72 km/h
        road_surface(2,i) = (0.04*(1-cos(7*pi*sim_time(i))));
    else
        road_surface(2,i) = 0;  
    end
end

road_surface = timeseries(road_surface(2,:),sim_time);

% figure('Name','Road profile')
% plot(road_surface)
% xlabel('Time (s)')
% ylabel('zr [m]')
% title('Road surface displacement')

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
Kp = pid.Kp + 1e5;
Kd = pid.Kd + 1e2;
Ki = pid.Ki + 2*1e6;

pid_2 = pidtune(qcar(5,2),'PID');
Kp_2 = pid_2.Kp + 1.6e2;
Kd_2 = pid_2.Kd;
Ki_2 = pid_2.Ki + 6*1e3;

%% Controllability

% the controllability matrix must be full ranck
rank(ctrb(A,B))

%% LQR Control law
% Q = diag([1*1e8, 1*1e7, 1, 1]);

Q = diag([1e10, 1e8, 1, 1]);
R = 1;

% feedback gain vector
K = lqr( A, B(:,2), Q, R );

%% Start Simulation
%------------------------------------------------------------------------------
fprintf( 'Starting Simulation\n' )

tic; % start measuring time

% Run the simulation
model_sim = sim('quarter_car_Active_Suspension_model');

elapsed_time_simulation = toc; % stop measuring time

fprintf( 'Simulation completed\n' )
fprintf( 'The total simulation time was %.2f seconds\n', elapsed_time_simulation)

%% Extreact results

% road profile 
road_profile = model_sim.road_profile;

figure('Name','Road profile'); hold on;
plot(road_profile)
title('Road surface displacement')
xlabel('$Time [s]$')
ylabel('$z_r [m]$')
xlim([0 3])

% state space variable
state_passive = model_sim.state.state_passive;
state_LQR = model_sim.state.state_LQR;
state_PID = model_sim.state.state_PID;

figure('Name','Suspension travel'); hold on;
plot(state_passive.zs_zus)
plot(state_LQR.zs_zus)
plot(state_PID.zs_zus)
legend({'passive','LQR','PID'})
title('Suspension travel')
ylabel('$z_s - z_{us} [m]$')
xlabel('$Time [s]$')
xlim([0 3])

figure('Name','Sprung mass velocity'); hold on;
plot(state_passive.zs_d)
plot(state_LQR.zs_d)
plot(state_PID.zs_d)
legend({'passive','LQR','PID'})
title('Sprung mass velocity')
ylabel('$\dot{z_s}$ [m/s]')
xlabel('$Time [s]$')
xlim([0 3])

figure('Name','Tire deflection'); hold on;
plot(state_passive.zus_zr)
plot(state_LQR.zus_zr)
plot(state_PID.zus_zr)
legend({'passive','LQR','PID'})
title('Tire deflection')
ylabel('$z_{us} - z_r$ [m/s]')
xlabel('$Time [s]$')
xlim([0 3])

figure('Name','Unsprung mass velocity'); hold on;
plot(state_passive.zus_d)
plot(state_LQR.zus_d)
plot(state_PID.zus_d)
legend({'passive','LQR','PID'})
title('Unsprung mass velocity')
xlabel('Time (s)')
ylabel('$\dot{z_{us}}$ [m/s]')
xlabel('$Time [s]$')
xlim([0 3])

% sprung mass acceleration
sprung_mass_a = model_sim.sprung_mass_acc;

figure('Name','Sprung mass acceleration'); hold on;
plot(sprung_mass_a.zs_dd_passive)
plot(sprung_mass_a.zs_dd_LQR)
plot(sprung_mass_a.zs_dd_PID)
legend({'passive','LQR','PID'})
title('Sprung mass acceleration')
ylabel('$\ddot{z_s} [m/s^2]$')
xlabel('$Time [s]$')
xlim([0 3])

% sprung mass motion
sprung_mass_motion = model_sim.ms_motion;

figure('Name','Sprung mass motion'); hold on;
plot(sprung_mass_motion.zs_passive)
plot(sprung_mass_motion.zs_LQR)
plot(sprung_mass_motion.zs_PID)
legend({'passive','LQR','PID'})
title('Sprung mass motion')
ylabel('$z_s [m]$')
xlabel('$Time [s]$')
xlim([0 3])

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

ms_motion_passive = sprung_mass_motion.zs_passive.Data;
ms_motion_LQR = sprung_mass_motion.zs_LQR.Data;
ms_motion_PID = sprung_mass_motion.zs_PID.Data;

%% RMS performance index

% Suspension travel
PR_RMS.rms_sp_LQR_index = (1 - abs(rms(sp_LQR))/abs(rms(sp_passive)))*100;
PR_RMS.rms_sp_PID_index = (1 - abs(rms(sp_PID))/abs(rms(sp_passive)))*100;

% Sprung mass velocity
PR_RMS.rms_msd_LQR_index = (1 - abs(rms(msd_LQR))/abs(rms(msd_passive)))*100;
PR_RMS.rms_msd_PID_index = (1 - abs(rms(msd_PID))/abs(rms(msd_passive)))*100;

% Tire deflection
PR_RMS.rms_td_LQR_index = (1 - abs(rms(td_LQR))/abs(rms(td_passive)))*100;
PR_RMS.rms_td_PID_index = (1 - abs(rms(td_PID))/abs(rms(td_passive)))*100;

% Unsprung mass velocity
PR_RMS.rms_musd_LQR_index = (1 - abs(rms(musd_LQR))/abs(rms(musd_passive)))*100;
PR_RMS.rms_musd_PID_index = (1 - abs(rms(musd_PID))/abs(rms(musd_passive)))*100;

% Sprung mass acceleration
PR_RMS.rms_msdd_LQR_index = (1 - abs(rms(msdd_LQR))/abs(rms(msdd_passive)))*100;
PR_RMS.rms_msdd_PID_index = (1 - abs(rms(msdd_PID))/abs(rms(msdd_passive)))*100;

% Sprung mass motion
PR_RMS.rms_ms_motion_LQR_index = (1 - abs(rms(ms_motion_LQR))/abs(rms(ms_motion_passive)))*100;
PR_RMS.rms_ms_motion_PID_index = (1 - abs(rms(ms_motion_PID))/abs(rms(ms_motion_passive)))*100;

PR_RMS

%% Overshoot performance index

% Suspension travel
PR_OVER.over_sp_LQR_index = (1 - max(abs(sp_LQR))/max(abs(sp_passive)))*100;
PR_OVER.over_sp_PID_index = (1 - max(abs(sp_PID))/max(abs(sp_passive)))*100;

% Sprung mass velocity
PR_OVER.over_msd_LQR_index = (1 - max(abs(msd_LQR))/max(abs(msd_passive)))*100;
PR_OVER.over_msd_PID_index = (1 - max(abs(msd_PID))/max(abs(msd_passive)))*100;

% Tire deflection
PR_OVER.over_td_LQR_index = (1 - max(abs(td_LQR))/max(abs(td_passive)))*100;
PR_OVER.over_td_PID_index = (1 - max(abs(td_PID))/max(abs(td_passive)))*100;

% Unsprung mass velocity
PR_OVER.over_musd_LQR_index = (1 - max(abs(musd_LQR))/max(abs(musd_passive)))*100;
PR_OVER.over_musd_PID_index = (1 - max(abs(musd_PID))/max(abs(musd_passive)))*100;

% Sprung mass acceleration
PR_OVER.over_msdd_LQR_index = (1 - max(abs(msdd_LQR))/max(abs(msdd_passive)))*100;
PR_OVER.over_msdd_PID_index = (1 - max(abs(msdd_PID))/max(abs(msdd_passive)))*100;

% Sprung mass motion
PR_OVER.over_ms_motion_LQR_index = (1 - max(abs(ms_motion_LQR))/max(abs(ms_motion_passive)))*100;
PR_OVER.over_ms_motion_PID_index = (1 - max(abs(ms_motion_PID))/max(abs(ms_motion_passive)))*100;

PR_OVER

%% Stability analysis

bode_plot

