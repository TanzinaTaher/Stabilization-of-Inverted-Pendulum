%% Pole Placement for Set 1 and Set 2 
clear; clc; close all;

%% -------------------------------
% Set 1: Long Pendulum Parameters
% -------------------------------
A1 = [ 0      1       0         0;
       0  -0.0467  0.9106      0;
       0      0       0         1;
       0   0.0611 -14.0284      0 ];

B1 = [ 0;
       0.9340;
       0;
      -1.2222 ];

%% -------------------------------
% Set 2: Short Pendulum Parameters
% -------------------------------
A2 = [ 0      1       0         0;
       0  -0.0775  0.7874      0;
       0      0       0         1;
       0   0.1859 -25.4203      0 ];

B2 = [ 0;
       1.5499;
       0;
      -3.7177 ];

%% Desired Closed-loop Poles
p_des = [ -4 + 5.4553j;
          -4 - 5.4553j;
          -20;
          -40 ];

%% Gains
K1 = place(A1, B1, p_des);
K2 = place(A2, B2, p_des);

%% Simulation Setup
dt = 0.001;
t  = 0:dt:5;
x0 = [0; 0; 0.15; 0];

%% Sim Set 1
A1_cl = A1 - B1*K1;
sys1 = ss(A1_cl, [], eye(4), []);
[x1, ~] = initial(sys1, x0, t);

theta1 = x1(:,3);
x_cart1 = x1(:,1);
u1 = -(K1 * x1.').';

%% Sim Set 2
A2_cl = A2 - B2*K2;
sys2 = ss(A2_cl, [], eye(4), []);
[x2, ~] = initial(sys2, x0, t);

theta2 = x2(:,3);
x_cart2 = x2(:,1);
u2 = -(K2 * x2.').';

%% Print Gains
fprintf('\n====================\n');
fprintf('   Pole Placement Gains\n');
fprintf('====================\n');

fprintf('\nSet 1 (Long Pendulum):\n');
fprintf('K1 = [ %.4f   %.4f   %.4f   %.4f ]\n', K1(1), K1(2), K1(3), K1(4));

fprintf('\nSet 2 (Short Pendulum):\n');
fprintf('K2 = [ %.4f   %.4f   %.4f   %.4f ]\n', K2(1), K2(2), K2(3), K2(4));

fprintf('\n');


%% =======================
% FIGURE 1: Pendulum Angle 
%% =======================
figure;
subplot(3,1,1);
plot(t, theta1, 'LineWidth', 1.8);
title('Pendulum Angle - Set 1 (Long Pendulum)');
xlabel('Time (s)'); ylabel('\theta(t) (rad)'); grid on;

subplot(3,1,2);
plot(t, theta2, 'LineWidth', 1.8, 'Color', [0.9 0.1 0.1]);
title('Pendulum Angle - Set 2 (Short Pendulum)');
xlabel('Time (s)'); ylabel('\theta(t) (rad)'); grid on;

subplot(3,1,3);
plot(t, theta1, 'LineWidth', 1.8); hold on;
plot(t, theta2, 'LineWidth', 1.8, 'Color',[0.9 0.1 0.1]);
title('Combined Comparison');
xlabel('Time (s)'); ylabel('\theta(t) (rad)');
legend('Set 1','Set 2');
grid on;

%% ======================
% FIGURE 2: Cart Position 
%% ======================
figure;
subplot(3,1,1);
plot(t, x_cart1, 'LineWidth', 1.8);
title('Cart Position - Set 1 (Long Pendulum)');
xlabel('Time (s)'); ylabel('x(t) (m)'); grid on;

subplot(3,1,2);
plot(t, x_cart2, 'LineWidth', 1.8, 'Color',[0.9 0.1 0.1]);
title('Cart Position - Set 2 (Short Pendulum)');
xlabel('Time (s)'); ylabel('x(t) (m)'); grid on;

subplot(3,1,3);
plot(t, x_cart1, 'LineWidth', 1.8); hold on;
plot(t, x_cart2, 'LineWidth', 1.8, 'Color',[0.9 0.1 0.1]);
title('Combined Comparison');
xlabel('Time (s)'); ylabel('x(t) (m)');
legend('Set 1','Set 2');
grid on;

%% =======================
% FIGURE 3: Control Effort 
%% =======================
figure;
subplot(3,1,1);
plot(t, u1, 'LineWidth', 1.8);
title('Control Effort - Set 1 (Long Pendulum)');
xlabel('Time (s)'); ylabel('u(t) (N)'); grid on;

subplot(3,1,2);
plot(t, u2, 'LineWidth', 1.8, 'Color',[0.9 0.1 0.1]);
title('Control Effort - Set 2 (Short Pendulum)');
xlabel('Time (s)'); ylabel('u(t) (N)'); grid on;

subplot(3,1,3);
plot(t, u1, 'LineWidth', 1.8); hold on;
plot(t, u2, 'LineWidth', 1.8, 'Color',[0.9 0.1 0.1]);
title('Combined Comparison');
xlabel('Time (s)'); ylabel('u(t) (N)');
legend('Set 1','Set 2');
grid on;
