%% ================================================================
%   Comparison of Pole Placement vs. LQR — Set 1 & Set 2
%   Each Figure Has 3 Subplots: PP, LQR, Comparison
% ================================================================
clear; clc; close all;

dt = 0.001;
t  = 0:dt:6;
x0 = [0; 0; 0.15; 0];   % initial tilt: 0.15 rad

%% =======================
%   Set 1 — Long Pendulum
%% =======================
A1 = [ 0      1       0         0;
       0  -0.0467  0.9106      0;
       0      0       0         1;
       0   0.0611 -14.0284      0 ];

B1 = [ 0;
       0.9340;
       0;
      -1.2222 ];

%% Desired poles (same for both sets)
p_des = [ -4 + 5.4553j;
          -4 - 5.4553j;
          -20;
          -40 ];

%% Pole Placement Gain (Set 1)
K1_PP = place(A1, B1, p_des);

%% LQR Gain (Set 1)
Q = diag([40 20 300 40]);
R = 12;
K1_LQR = lqr(A1, B1, Q, R);

%% Simulate Set 1 — PP
A1_cl_PP = A1 - B1*K1_PP;
sys1_PP  = ss(A1_cl_PP, [], eye(4), []);
[x1_PP, ~] = initial(sys1_PP, x0, t);
theta1_PP = x1_PP(:,3);
x_cart1_PP = x1_PP(:,1);
u1_PP = -(K1_PP * x1_PP.').';

%% Simulate Set 1 — LQR
A1_cl_LQR = A1 - B1*K1_LQR;
sys1_LQR = ss(A1_cl_LQR, [], eye(4), []);
[x1_LQR, ~] = initial(sys1_LQR, x0, t);
theta1_LQR = x1_LQR(:,3);
x_cart1_LQR = x1_LQR(:,1);
u1_LQR = -(K1_LQR * x1_LQR.').';

%% =======================
%   Set 2 — Short Pendulum
%% =======================
A2 = [ 0      1       0         0;
       0  -0.0775  0.7874      0;
       0      0       0         1;
       0   0.1859 -25.4203      0 ];

B2 = [ 0;
       1.5499;
       0;
      -3.7177 ];

%% Gains — Set 2
K2_PP = place(A2, B2, p_des);
K2_LQR = lqr(A2, B2, Q, R);

%% Simulate Set 2 — PP
A2_cl_PP = A2 - B2*K2_PP;
sys2_PP = ss(A2_cl_PP, [], eye(4), []);
[x2_PP, ~] = initial(sys2_PP, x0, t);
theta2_PP = x2_PP(:,3);
x_cart2_PP = x2_PP(:,1);
u2_PP = -(K2_PP * x2_PP.').';

%% Simulate Set 2 — LQR
A2_cl_LQR = A2 - B2*K2_LQR;
sys2_LQR = ss(A2_cl_LQR, [], eye(4), []);
[x2_LQR, ~] = initial(sys2_LQR, x0, t);
theta2_LQR = x2_LQR(:,3);
x_cart2_LQR = x2_LQR(:,1);
u2_LQR = -(K2_LQR * x2_LQR.').';

%% ============================================================
%               FIGURE 1 — ANGLE (Set 1)
%% ============================================================
figure;
subplot(3,1,1);
plot(t, theta1_PP, 'LineWidth', 2);
title('Set 1 — Pole Placement: Pendulum Angle');
ylabel('\theta(t) (rad)'); grid on;

subplot(3,1,2);
plot(t, theta1_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 1 — LQR: Pendulum Angle');
ylabel('\theta(t) (rad)'); grid on;

subplot(3,1,3);
plot(t, theta1_PP, 'LineWidth', 2); hold on;
plot(t, theta1_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 1 — Angle Comparison');
ylabel('\theta(t)'); xlabel('Time (s)');
legend('PP','LQR'); grid on;


%% ============================================================
%               FIGURE 2 — CART POSITION (Set 1)
%% ============================================================
figure;
subplot(3,1,1);
plot(t, x_cart1_PP, 'LineWidth', 2);
title('Set 1 — Pole Placement: Cart Position');
ylabel('x(t) (m)'); grid on;

subplot(3,1,2);
plot(t, x_cart1_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 1 — LQR: Cart Position');
ylabel('x(t) (m)'); grid on;

subplot(3,1,3);
plot(t, x_cart1_PP, 'LineWidth', 2); hold on;
plot(t, x_cart1_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 1 — Cart Position Comparison');
ylabel('x(t)'); xlabel('Time (s)');
legend('PP','LQR'); grid on;


%% ============================================================
%               FIGURE 3 — CONTROL EFFORT (Set 1)
%% ============================================================
figure;
subplot(3,1,1);
plot(t, u1_PP, 'LineWidth', 2);
title('Set 1 — Pole Placement: Control Effort');
ylabel('u(t) (N)'); grid on;

subplot(3,1,2);
plot(t, u1_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 1 — LQR: Control Effort');
ylabel('u(t) (N)'); grid on;

subplot(3,1,3);
plot(t, u1_PP, 'LineWidth', 2); hold on;
plot(t, u1_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 1 — Control Effort Comparison');
ylabel('u(t)'); xlabel('Time (s)');
legend('PP','LQR'); grid on;


%% ============================================================
%               FIGURE 4 — ANGLE (Set 2)
%% ============================================================
figure;
subplot(3,1,1);
plot(t, theta2_PP, 'LineWidth', 2);
title('Set 2 — Pole Placement: Pendulum Angle');
ylabel('\theta(t)'); grid on;

subplot(3,1,2);
plot(t, theta2_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 2 — LQR: Pendulum Angle');
ylabel('\theta(t)'); grid on;

subplot(3,1,3);
plot(t, theta2_PP, 'LineWidth', 2); hold on;
plot(t, theta2_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 2 — Angle Comparison');
ylabel('\theta(t)'); xlabel('Time (s)');
legend('PP','LQR'); grid on;


%% ============================================================
%               FIGURE 5 — CART POSITION (Set 2)
%% ============================================================
figure;
subplot(3,1,1);
plot(t, x_cart2_PP, 'LineWidth', 2);
title('Set 2 — Pole Placement: Cart Position');
ylabel('x(t)'); grid on;

subplot(3,1,2);
plot(t, x_cart2_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 2 — LQR: Cart Position');
ylabel('x(t)'); grid on;

subplot(3,1,3);
plot(t, x_cart2_PP, 'LineWidth', 2); hold on;
plot(t, x_cart2_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 2 — Cart Position Comparison');
ylabel('x(t)'); xlabel('Time (s)');
legend('PP','LQR'); grid on;


%% ============================================================
%               FIGURE 6 — CONTROL EFFORT (Set 2)
%% ============================================================
figure;
subplot(3,1,1);
plot(t, u2_PP, 'LineWidth', 2);
title('Set 2 — Pole Placement: Control Effort');
ylabel('u(t) (N)'); grid on;

subplot(3,1,2);
plot(t, u2_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 2 — LQR: Control Effort');
ylabel('u(t) (N)'); grid on;

subplot(3,1,3);
plot(t, u2_PP, 'LineWidth', 2); hold on;
plot(t, u2_LQR, 'LineWidth', 2, 'Color',[0.8 0 0]);
title('Set 2 — Control Effort Comparison');
ylabel('u(t)'); xlabel('Time (s)');
legend('PP','LQR'); grid on;
