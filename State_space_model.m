%% IP02 Inverted Pendulum - State-Space from Quanser Data
% State: x = [x; x_dot; theta; theta_dot]
% Input: u = F (force on cart, N)

clear; clc;

%% Hardware parameters (Set 1: IP02 + 24" long pendulum)
M1      = 0.94;          % cart mass [kg]
m1      = 0.230;         % pendulum mass [kg]
Lp1     = 0.6413;        % pendulum full length [m]
l1      = 0.3302;        % pivot to COM [m]
Jp_cm1  = 7.8838e-3;     % inertia about COM [kg*m^2]
I1      = Jp_cm1 + m1*l1^2; % inertia about pivot [kg*m^2]
b1      = 0.050;         % cart viscous friction [N*s/m]
Bp1     = 0.0024;        % pendulum damping (not used in A,B)
g       = 9.81;          % gravity [m/s^2]

%% Common formulas for coefficients (as functions)
coeffs = @(M,m,l,I,b,g) deal( ...
    (M + m)*(I + m*l^2) - (m*l)^2, ...                        % p
    -(b*(I + m*l^2)), ...                                     % num_a22
    (m^2 * g * l^2), ...                                      % num_a23
    (m*l*b), ...                                              % num_a42
    -(m*g*l*(M + m)), ...                                     % num_a43
    (I + m*l^2), ...                                          % num_b2
    -(m*l) );                                                 % num_b4

%% Coefficients and state-space (Set 1)
[p1, num_a22_1, num_a23_1, num_a42_1, num_a43_1, num_b2_1, num_b4_1] = ...
    coeffs(M1,m1,l1,I1,b1,g);

a22_1 = num_a22_1 / p1;
a23_1 = num_a23_1 / p1;
a42_1 = num_a42_1 / p1;
a43_1 = num_a43_1 / p1;
b2_1  = num_b2_1  / p1;
b4_1  = num_b4_1  / p1;

A1 = [ 0      1       0       0;
       0    a22_1   a23_1    0;
       0      0       0       1;
       0    a42_1   a43_1    0 ];

B1 = [ 0;
       b2_1;
       0;
       b4_1 ];

C = [ 1 0 0 0;      % output: cart position x
      0 0 1 0 ];    % output: pendulum angle theta

D = [ 0;
      0 ];

%% Hardware parameters (Set 2: IP02 + short pendulum)
M2      = 0.57;          % cart mass [kg]
m2      = 0.127;         % pendulum mass [kg]
Lp2     = 0.324;         % pendulum full length [m]
l2      = 0.170;         % pivot to COM [m]
Jp_cm2  = 1.66e-3;       % inertia about COM [kg*m^2]
I2      = 5.3303e-3;     % inertia about pivot [kg*m^2] (computed Jp_cm + m*l^2)
b2c     = 0.050;         % cart viscous friction [N*s/m]
Bp2     = 0.0024;        % pendulum damping (not used in A,B)

%% Coefficients and state-space (Set 2)
[p2, num_a22_2, num_a23_2, num_a42_2, num_a43_2, num_b2_2, num_b4_2] = ...
    coeffs(M2,m2,l2,I2,b2c,g);

a22_2 = num_a22_2 / p2;
a23_2 = num_a23_2 / p2;
a42_2 = num_a42_2 / p2;
a43_2 = num_a43_2 / p2;
b2_2  = num_b2_2  / p2;
b4_2  = num_b4_2  / p2;

A2 = [ 0      1       0       0;
       0    a22_2   a23_2    0;
       0      0       0       1;
       0    a42_2   a43_2    0 ];

B2 = [ 0;
       b2_2;
       0;
       b4_2 ];

%% Show matrices for Set 1
disp('--- Set 1: Long 24" Pendulum ---');
disp('A1 ='); disp(A1);
disp('B1 ='); disp(B1);
disp('C  ='); disp(C);
disp('D  ='); disp(D);

eigA1 = eig(A1);
disp('Eigenvalues of A1:');
disp(eigA1);

Co1 = ctrb(A1,B1);
Ob1 = obsv(A1,C);

fprintf('rank(ctrb(A1,B1)) = %d (out of %d states)\n', rank(Co1), size(A1,1));
fprintf('rank(obsv(A1,C))  = %d (out of %d states)\n\n', rank(Ob1), size(A1,1));

%% Show matrices for Set 2
disp('--- Set 2: Short Pendulum ---');
disp('A2 ='); disp(A2);
disp('B2 ='); disp(B2);

eigA2 = eig(A2);
disp('Eigenvalues of A2:');
disp(eigA2);

Co2 = ctrb(A2,B2);
Ob2 = obsv(A2,C);

fprintf('rank(ctrb(A2,B2)) = %d (out of %d states)\n', rank(Co2), size(A2,1));
fprintf('rank(obsv(A2,C))  = %d (out of %d states)\n', rank(Ob2), size(A2,1));

%% Create state-space models (for later controller design)
sys1 = ss(A1,B1,C,D);   % long pendulum model
sys2 = ss(A2,B2,C,D);   % short pendulum model
