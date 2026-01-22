%% Model Reference Control Implementation (CONTD. Part 2)

%% ================= Simulation Settings =================

clear; 
clc;

% Velocity Command Signal, v
v_step = 0;                    % No base command
step1 = 0;
step2 = 0;

% Time Interval 
t_on  = 0;
t_off = 0;
t_sim = 20;                    % Simulation duration in sec

%% ================= Plant & Reference Model Dynamics =================

% Declaring the System Coefficient Matrices as Global Variables
global Ap Bp Cp Dp Am Bm Cm Dm P;


% Defining Physical Parameters of the Plant
g = 9.81;                                  % Gravity
L = 1;                                     % Cable Suspension Length Fixed to 1m
zeta = 0;                                  % Plant Damping Ratio
wn = sqrt(g/L);                            % Plant Natural Frequency


% Defining Plant State-Space Matrices
Ap = [0   1;  -wn^2   -2*zeta*wn];             % Plant Coefficient Matrix
Bp = [0;  1];                                  % Input Coefficient Vector
Cp = eye(2);                                   % Full-State Feedback
Dp = [0;0];

% Defining Model Parameters
wn_model = sqrt(g);                           % Model Frequency of 1m Cable Length
prompt2 = 'Input the Model Damping Ratio: ';  % Input Damping Ratios 0.1, 0.5 & 0.8
zeta_model = input(prompt2);                  % Model Damping Ratio   

% Defining Model State-Space Matrices
Am = [0 1; -wn_model^2 -2*zeta_model*wn_model];  % Model Coefficient Matrix
Bm = [0; 1];
Cm = eye(2);
Dm = [0;0];

%% ========== Computing P-Matrix using User-Defined Q-Matrix ==========

% Defining Positive-Definite, Symmetric, Real Q-matrix
Q = eye(2);                   

% Symbolic Representation & Computation of P-matrix
syms p11 p12 p22 real
P = [p11 p12; p12 p22];
P_MatEqn = Q + Am'*P + P*Am;
[p11_symb, p12_symb, p22_symb] = solve(P_MatEqn == 0, [p11,p12,p22]);

% Computed P-Matrix converted to its Numeric Values
P = double([p11_symb p12_symb; p12_symb p22_symb]); 

%% ================= Error Dynamics & Control Law Derivation =================

syms u v e1 e2 x1 x2 real
e = [e1; e2];                           % Error Vector
x = [x1; x2];                           % Plant State Vector
M = (Am - Ap)*x + Bm*v - Bp*u;          % Core MRC Error Dynamics

LyapunovFunc = 2*e'*P*M;
u_func = solve(LyapunovFunc == 0, u)    % Expression for Control Effort

%% ================= Modeling External Disturbance =================

% Defining Disturbance Parameters
t_disturb  = 2.0;      % Impulse Application Time (s)
d_duration = 0.02;     % Impulse Duration (s)
d_mag      = 0.5;      % Disturbance magnitude (m/s^-2)


% Defining Disturbance Signal Vector for SIMULINK
Ts = 0.001;
t_vec = 0:Ts:t_sim;
disturbance = zeros(size(t_vec));
idx = (t_vec >= t_disturb) & (t_vec <= t_disturb + d_duration);
disturbance(idx) = d_mag;

disturbance_signal = [t_vec(:), disturbance(:)];
assignin('base','disturbance_signal',disturbance_signal);

%% ========= SIMULINK Model Execution & Retrieving SIMULINK Output Data =========

% SIMULINK Execution
sim('Disturbance_Rejection');


% System Response Extraction
xd    = simout_xd.Data;
x     = simout_x.Data;
errs  = simout_e.Data;
times = simout_x.Time;

%% ===== Visualization of Plant Response in Simulated & Experimental Setup ======

% Plant Oscillations in Simulated Environment
x2_real = x(:,2);                              % x2 = (the theta*L state)
deflAngle = [times, x2_real/L]; 
deflections = deflAngle(:, 2)*1000;        % Payload Deflections in mm


% Plant Oscillations in Experimental Setup due to small disturbance
load ModelC_SmallPerturbations.mat
TimeC_Small = ModelC_Small_Perturb.TimeYdirsec;
DeflectionsC_Small = ModelC_Small_Perturb.YPayloadDeflectionmm;


% Visualization
Fig1 = figure();
subplot(2,1,1)
plot(times, deflections, "LineWidth", 1.75, "Color","#d1048b",...
     "Marker",".","MarkerSize", 13, 'MarkerEdgeColor', "#0066cc",...
     'MarkerIndices', 1:2:length(deflections))

xlabel("Time (sec)", "FontWeight","bold", "FontSize", 11)
ylabel("Plant Deflections (mm)", "FontWeight","bold", "FontSize", 11)
title("Plant Response to Disturbance in Simulation","FontWeight","bold",...
      "FontSize", 11);
grid on
grid minor


subplot(2,1,2)
plot(TimeC_Small, DeflectionsC_Small, 'LineWidth',1.5, 'Color',...
    'c','Marker','.',"MarkerSize", 12, 'MarkerIndices',...
     1:2:length(DeflectionsC_Small))

xlim([0 20]);
title("Plant Response to Disturbance in Experimental Setup","FontWeight","bold",...
      "FontSize", 11);
xlabel("Time (sec)", "FontWeight","bold", "FontSize", 11)
ylabel("Plant Deflections (mm)", "FontWeight","bold", "FontSize", 11)
grid on
grid minor