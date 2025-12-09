%% LQR Controller Design and Simulation for Set 1 and Set 2
clear; clc; close all;

%% -----------------------------------------
% Set 1: Long Pendulum (24-inch) Parameters
% -----------------------------------------
A1 = [ 0      1       0         0;
       0  -0.0467   0.9106      0;
       0      0       0         1;
       0   0.0611  -14.0284     0 ];

B1 = [ 0;
       0.9340;
       0;
      -1.2222 ];

%% -----------------------------------------
% Set 2: Short Pendulum Parameters
% -----------------------------------------
A2 = [ 0      1       0         0;
       0  -0.0775   0.7874      0;
       0      0       0         1;
       0   0.1859  -25.4203     0 ];

B2 = [ 0;
       1.5499;
       0;
      -3.7177 ];

%% Initial Condition & Time
x0 = [0; 0; 0.15; 0];   % initial angle 0.15 rad
t  = 0:0.001:6;

%% Choose final Q and R (same structure used in Set 1 tuning)
Q_final = diag([40  20  300  40]);
R_final = 12;

%% Compute LQR Gains
[K1,~,~] = lqr(A1,B1,Q_final,R_final);
[K2,~,~] = lqr(A2,B2,Q_final,R_final);

fprintf("===== LQR Gains =====\n");
fprintf("Set 1 (Long Pendulum): K1 = [% .3f  % .3f  % .3f  % .3f]\n", K1);
fprintf("Set 2 (Short Pendulum): K2 = [% .3f  % .3f  % .3f  % .3f]\n", K2);

%% Closed-loop Systems
sys1 = ss(A1 - B1*K1, [], eye(4), []);
sys2 = ss(A2 - B2*K2, [], eye(4), []);

[x1,~] = initial(sys1,x0,t);
[x2,~] = initial(sys2,x0,t);

theta1 = x1(:,3);    theta2 = x2(:,3);
x_cart1 = x1(:,1);   x_cart2 = x2(:,1);

u1 = -(K1 * x1.').';
u2 = -(K2 * x2.').';

%% ========================
% FIGURE 1 — Pendulum Angle 
%% ========================
figure;
subplot(3,1,1);
plot(t, theta1, 'LineWidth', 1.8);
title('LQR Pendulum Angle — Set 1 (Long Pendulum)');
ylabel('\theta(t) (rad)'); grid on;

subplot(3,1,2);
plot(t, theta2, 'r', 'LineWidth', 1.8);
title('LQR Pendulum Angle — Set 2 (Short Pendulum)');
ylabel('\theta(t) (rad)'); grid on;

subplot(3,1,3);
plot(t, theta1, 'LineWidth', 1.8); hold on;
plot(t, theta2, 'r', 'LineWidth', 1.8);
title('LQR Pendulum Angle — Comparison');
xlabel('Time (s)'); ylabel('\theta(t) (rad)');
legend('Set 1','Set 2'); grid on;

%% =======================
% FIGURE 2 — Cart Position
%% =======================
figure;
subplot(3,1,1);
plot(t, x_cart1, 'LineWidth', 1.8);
title('LQR Cart Position — Set 1');
ylabel('x(t) (m)'); grid on;

subplot(3,1,2);
plot(t, x_cart2, 'r','LineWidth',1.8);
title('LQR Cart Position — Set 2');
ylabel('x(t) (m)'); grid on;

subplot(3,1,3);
plot(t, x_cart1, 'LineWidth',1.8); hold on;
plot(t, x_cart2, 'r','LineWidth',1.8);
title('LQR Cart Position — Comparison');
xlabel('Time (s)'); ylabel('x(t) (m)');
legend('Set 1','Set 2'); grid on;

%% ========================
% FIGURE 3 — Control Effort
%% ========================
figure;
subplot(3,1,1);
plot(t, u1, 'LineWidth',1.8);
title('LQR Control Effort — Set 1');
ylabel('u(t) (N)'); grid on;

subplot(3,1,2);
plot(t, u2, 'r','LineWidth',1.8);
title('LQR Control Effort — Set 2');
ylabel('u(t) (N)'); grid on;

subplot(3,1,3);
plot(t, u1, 'LineWidth',1.8); hold on;
plot(t, u2, 'r','LineWidth',1.8);
title('LQR Control Effort — Comparison');
xlabel('Time (s)'); ylabel('u(t) (N)');
legend('Set 1','Set 2'); grid on;
