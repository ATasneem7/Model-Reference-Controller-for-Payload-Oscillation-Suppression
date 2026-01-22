%% ---Model Reference Control Implementation (CONTD.)--- %%

%% Defining Simulation Input & Time

clc; 
clear;

% Velocity Command Signal, v
v_step = 0.2;                 % Maximum Step Size of the Velocity Command in m/s
step1 = v_step;               % Positive Velocity Step to Kick Start
step2 = -v_step;              % Negative Velocity Step to Decelerate

% Time Interval 
t_on = 0.25;                  % Velocity Pulse Activation Time 
t_off = 5;                    % Velocity Pulse Deactivation Time
t_sim = 20;                   % Simulation Time Window

%% Plant (Payload) & Reference Model Dynamics

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


% % Defining Model Parameters (Case #1: Unrealistic zeta)
% wn_model = sqrt(g);                         % Model Frequency of 1m Cable Length
% prompt2='Input the Model Damping Ratio: ';  % Input Damping Ratios >=1
% zeta_model = input(prompt2);                % Model Damping Ratio


% Defining Model Parameters (Case #2: Unrealistic Frequency)
zeta_model = 0.5;                           % Damping Ratio Fixed to 0.5
prompt2 = 'Input the Model Frequency: ';    % Input Model Frequency > wn
wn_model = input(prompt2);


% Defining Model State-Space Matrices
Am = [0 1; -wn_model^2 -2*zeta_model*wn_model];  % Model Coefficient Matrix
Bm = [0; 1];
Cm = eye(2);
Dm = [0;0];

%% Computing P-Matrix using User-Defined Q-Matrix

% Defining Positive-Definite, Symmetric, Real Q-matrix
Q = eye(2);                   

% Symbolic Representation & Computation of P-matrix
syms p11 p12 p22 real
P = [p11 p12; p12 p22];
P_MatEqn = Q + Am'*P + P*Am;
[p11_symb, p12_symb, p22_symb] = solve(P_MatEqn == 0, [p11,p12,p22]);

% Computed P-Matrix converted to its Numeric Values
P = double([p11_symb p12_symb; p12_symb p22_symb]);  

%% Error Dynamics & Control Law Derivation

syms u v e1 e2 x1 x2 real
e = [e1; e2];                         % Error Vector
x = [x1; x2];                         % Plant State Vector
M = (Am - Ap)*x + Bm*v - Bp*u;        % Core MRC Error Dynamics

LyapunovFunc = 2*e'*P*M;
u_func = solve(LyapunovFunc == 0, u)  % Analytical Expression for Control Effort 

%% SIMULINK Model Execution & Retrieving SIMULINK Output Data

% SIMULINK Execution
sim('Control_System_Framework');

% System Response Extraction
xd = simout_xd.Data;          % Model Plant States
x = simout_x.Data;            % Actual Plant States
u = simout_u.Data;            % Control Efforts
times = simout_u.Time;        % Simulation Time Vector
errs = simout_e.Data;         % Tracking Error

%% SIMULINK Animation Execution

% State data for running animation 
x2_real = x(:,2);                              % x2 = (the theta*L state)
deflAngle = [times, x2_real/L];                % deflAngle = [times, x2/L];
trolleyPos = zeros(length(times),1);
for aa = 2:length(times)
    trolleyPos(aa) = trapz(times(aa-1:aa),u(aa-1:aa)) + trolleyPos(aa-1);
end
xt = [times, trolleyPos];

% Animation Execution
sim('Payload_Motion_Animation');

%% Visualization of System Response in Simulated Environment & Experimental Setup

% Plant Oscillations in Simulated Environment
deflections = deflAngle(:, 2)*1000;                % Payload Deflections in mm

% Plant Oscillations in Experimental Setup
load Unrealistic_Model_Omega3.53.mat
TimeOmega2 = Model_Omega2.TimeYdirsec;
DeflectionsOmega2 = Model_Omega2.YPayloadDeflectionmm;

Fig1 = figure();
subplot(2,1,1)
plot(times, deflections, "LineWidth", 1.75, "Color","#d1048b",...
     "Marker",".","MarkerSize", 13, 'MarkerEdgeColor', "#0066cc",...
     'MarkerIndices', 1:2:length(deflections))

xlim([0 15]);
xlabel("Time (sec)", "FontWeight","bold", "FontSize", 11)
ylabel("Deflection (mm)", "FontWeight","bold", "FontSize", 11)
title("Plant Deflection Time History in Simulation","FontWeight","bold",...
      "FontSize", 11);
grid on
grid minor


subplot(2,1,2)
plot(TimeOmega2, DeflectionsOmega2, 'LineWidth',1.5, 'Color',...
    'm','Marker','.',"MarkerSize", 12, 'MarkerIndices',...
     1:2:length(DeflectionsOmega2))

xlim([0 15]);
title('Plant Deflection Time History in Experimental Setup',"FontWeight","bold",...
      "FontSize", 11);
xlabel("Time (sec)", "FontWeight","bold", "FontSize", 11)
ylabel("Deflection (mm)", "FontWeight","bold", "FontSize", 11)
grid on
grid minor