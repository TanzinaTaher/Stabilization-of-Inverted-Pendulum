%% PID tuning for both parameter sets of the inverted pendulum
clear; clc; close all;

%% ============================================
%  Parameter Set 1 (Long Pendulum)
% =============================================
m1 = 0.230;
l1 = 0.3302;
g  = 9.81;
Jcm1 = 7.8838e-3;
J1 = Jcm1 + m1*l1^2;

a1 = (m1*g*l1)/J1;
b1 = 1/J1;

A1 = [0 1; a1 0];
B1 = [0; b1];

%% ============================================
%  Parameter Set 2 (Short Pendulum)
% =============================================
m2 = 0.127;
l2 = 0.170;
Jcm2 = 1.66e-3;
J2 = Jcm2 + m2*l2^2;

a2 = (m2*g*l2)/J2;
b2 = 1/J2;

A2 = [0 1; a2 0];
B2 = [0; b2];

%% Common simulation settings
dt = 0.001;
t_end = 5;
t = 0:dt:t_end;
N = numel(t);

theta0 = 0.15;
x0 = [theta0; 0];

%% ===============================
%  Gains for all three modes
% ===============================
% Set 1
P1 = struct('Kp',0.6, 'Ki',0, 'Kd',0);
PD1 = struct('Kp',1.0,'Ki',0, 'Kd',0.06);
PID1 = struct('Kp',1.2,'Ki',0.30,'Kd',0.09);

% Set 2
P2 = struct('Kp',0.4, 'Ki',0, 'Kd',0);
PD2 = struct('Kp',0.9,'Ki',0, 'Kd',0.04);
PID2 = struct('Kp',1.1,'Ki',0.25,'Kd',0.06);

%% Helper function for simulation
simulatePID = @(A,B,K) pidSim(A,B,x0,t,dt,K);

%% -------------------
%  SIMULATE EVERYTHING
%% -------------------
[xP1, uP1]     = simulatePID(A1,B1,P1);
[xPD1, uPD1]   = simulatePID(A1,B1,PD1);
[xPID1, uPID1] = simulatePID(A1,B1,PID1);

[xP2, uP2]     = simulatePID(A2,B2,P2);
[xPD2, uPD2]   = simulatePID(A2,B2,PD2);
[xPID2, uPID2] = simulatePID(A2,B2,PID2);

%% ============================================
%  FIGURE 1 — P-Only Comparison (2 subplots)
% =============================================
figure;
subplot(2,1,1);
plot(t, xP1(:,1),'LineWidth',1.8); grid on;
title('Set 1 — P Control'); ylabel('\theta(t)');

subplot(2,1,2);
plot(t, xP2(:,1),'LineWidth',1.8); grid on;
title('Set 2 — P Control'); ylabel('\theta(t)'); xlabel('Time (s)');

%% ============================================
%  FIGURE 2 — PD Comparison (2 subplots)
% =============================================
figure;
subplot(2,1,1);
plot(t, xPD1(:,1),'LineWidth',1.8); grid on;
title('Set 1 — PD Control'); ylabel('\theta(t)');

subplot(2,1,2);
plot(t, xPD2(:,1),'LineWidth',1.8); grid on;
title('Set 2 — PD Control'); ylabel('\theta(t)'); xlabel('Time (s)');

%% ============================================
%  FIGURE 3 — PID Comparison (2 subplots)
% =============================================
figure;
subplot(2,1,1);
plot(t, xPID1(:,1),'LineWidth',1.8); grid on;
title('Set 1 — PID Control'); ylabel('\theta(t)');

subplot(2,1,2);
plot(t, xPID2(:,1),'LineWidth',1.8); grid on;
title('Set 2 — PID Control'); ylabel('\theta(t)'); xlabel('Time (s)');

%% ============================================
%  FIGURE 4 — Angle Comparison (Final PID)
% =============================================
figure;
plot(t, xPID1(:,1),'LineWidth',1.8); hold on;
plot(t, xPID2(:,1),'LineWidth',1.8);
ylabel('\theta(t) (rad)');
xlabel('Time (s)');
title('Final PID Angle Comparison — Set 1 vs Set 2');
legend('Set 1 (Long)','Set 2 (Short)');
grid on;

%% ============================================
%  FIGURE 5 — Control Effort Comparison
% =============================================
figure;
plot(t, uPID1,'LineWidth',1.8); hold on;
plot(t, uPID2,'LineWidth',1.8);
ylabel('u(t)');
xlabel('Time (s)');
title('PID Control Effort Comparison — Set 1 vs Set 2');
legend('Set 1 (Long)','Set 2 (Short)');
grid on;

%% -----------------
%  PID Simulation Function
%% -----------------
function [x_hist, u_hist] = pidSim(A,B,x0,t,dt,K)
N = numel(t);
x_hist = zeros(N,2);
u_hist = zeros(N,1);
x = x0;
e_prev = 0;
e_int  = 0;

for k = 1:N
    theta = x(1);
    e = -theta;

    de = (e - e_prev)/dt;
    e_int = e_int + e*dt;

    u = K.Kp*e + K.Ki*e_int + K.Kd*de;

    x_hist(k,:) = x';
    u_hist(k)   = u;

    x = x + dt*(A*x + B*u);
    e_prev = e;
end
end
