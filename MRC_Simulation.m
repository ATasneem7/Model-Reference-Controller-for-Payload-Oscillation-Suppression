%% -----Model Reference Control Implementation on Payload Oscillation----- %%

% Simulation Initialization
clear;
clc;

%% Defining Simulation Input & Time

% Defining Velocity Command Signal, v
v_step = 0.2;                    % Maximum Step Size of the Velocity Command in m/s
step1 = v_step;                  % Positive Velocity Step to Kick Start
step2 = -v_step;                 % Negative Velocity Step to Decelerate


% Defining Time Interval for Input Application & Simulation Run
t_on = 0;                        % Velocity Pulse Activation Time 
t_off = 5;                       % Velocity Pulse Deactivation Time
t_sim = 20;                      % Simulation Time Window

%% Plant (Payload) & Reference Model Dynamics

% Declaring the System Coefficient Matrices as Global Variables
global Ap Bp Cp Dp Am Bm Cm Dm P;


% Defining Physical Parameters of the Plant
g = 9.81;                                            % Gravity
prompt1 = 'Input the Cable Suspension Length: ';     % Input Length between 0.6 ~ 1.5m
L = input(prompt1);                                  % Cable Suspension Length 
zeta = 0;                                            % Plant Damping Ratio
wn = sqrt(g/L);                                      % Plant (Payload) Natural Frequency


% Defining Plant State-Space Matrices
Ap = [0   1;  -wn^2   -2*zeta*wn];                   % Plant Coefficient Matrix
Bp = [0;  1];                                        % Input Coefficient Vector
Cp = eye(2);                                         % Full-State Feedback
Dp = [0;0];


% Defining Model Parameters 
wn_model = sqrt(g);                              % Model Frequency using 1m Cable Length
prompt2 = 'Input the Model Damping Ratio: ';     % Input Damping Ratios 0.1, 0.5 & 0.8
zeta_model = input(prompt2);                     % Model Damping Ratio


% Defining Model State-Space Matrices
Am = [0 1; -wn_model^2 -2*zeta_model*wn_model];  % Model Coefficient Matrix
Bm = [0; 1];
Cm = eye(2);
Dm = [0;0];

%% Computing P-Matrix using User-Defined Q-Matrix

prompt3 = 'Define the Q-matrix: ';
Q = input(prompt3)                         % Defining Q-matrix based on user-defined input


% Symbolic Representation & Computation of P-matrix
syms p11 p12 p22 real
P = [p11 p12; p12 p22];
P_MatEqn = Q + Am'*P + P*Am;
[p11_symb, p12_symb, p22_symb] = solve(P_MatEqn == 0, [p11,p12,p22]);


% Computed P-Matrix converted to its Numeric Values
P = double([p11_symb p12_symb; p12_symb p22_symb]) 

%% Error Dynamics & Control Law Derivation 

syms u v e1 e2 x1 x2 real
e = [e1; e2];                           % Error Vector
x = [x1; x2];                           % Plant State Vector
M = (Am - Ap)*x + Bm*v - Bp*u;          % Core MRC Error Dynamics

LyapunovFunc = 2*e'*P*M;
u_func = solve(LyapunovFunc == 0, u)    % Analytical Expression for Control Effort 

%% SIMULINK Model Execution & Retrieving SIMULINK Output Data

% SIMULINK Execution
sim('student_model_reference');


% System Response Extraction
xd = simout_xd.Data;          % Model Plant States
x = simout_x.Data;            % Actual Plant States
u = simout_u.Data;            % Control Efforts
times = simout_u.Time;        % Simulation Time Vector
errs = simout_e.Data;         % Tracking Error

%% SIMULINK Animation Execution

% State data for running animation 
x2_real = x(:,1);                              % x2 = (the theta*L state)
deflAngle = [times, x2_real/L];                % deflAngle = [times, x2/L];
trolleyPos = zeros(length(times),1);
for aa = 2:length(times)
    trolleyPos(aa) = trapz(times(aa-1:aa),u(aa-1:aa)) + trolleyPos(aa-1);
end
xt = [times, trolleyPos];


% Animation Execution
sim('student_model_animation');

%% Graphical Visualization of System Response & Controller Performance

% Visualizing Payload Response
deflections = (x2_real/L)*1000;        % Payload Deflections in mm
oscillations = x(:, 2) * 1000;         % Payload Oscillation in mm/s

fig1 = figure();
plot(times, deflections, "LineWidth", 1.75, "Color",[0.96,0.32,0.05],...
     "Marker",".","MarkerSize", 14, 'MarkerIndices', 1:2:length(x2_real))
hold on 
plot(times, oscillations, "LineWidth", 1.75, "Color","#6E026F",...
     "Marker",".","MarkerSize", 12, 'MarkerIndices', 1:2:length(x2_real))

xlabel("Time (sec)", "FontWeight","bold", "FontSize", 12)
ylabel("Plant States (mm)", "FontWeight","bold", "FontSize", 12)
title("Plant Response in Tracking Reference Model A",...
      "FontWeight","bold", "FontSize", 12);
legend("Payload Deflections (mm)", "Payload Oscillations (mm/s)")
grid on
grid minor
hold off


% Visualizing Tracking Error Convergence
defl_errs = (errs(:, 1))*1000;        % Deflection Tracking Error in mm             
oscl_errs = (errs(:, 2))*1000;        % Oscillation Tracking Error in mm/s 

fig2 = figure();
plot(times, defl_errs, "LineWidth", 1.85, "Color",[0.16,0.62,0.56],...
     "Marker",".","MarkerSize", 14, 'MarkerIndices', 1:2:length(defl_errs))
hold on
plot(times, oscl_errs, "LineWidth", 1.85, "Color","#F16D34",...
     "Marker",".","MarkerSize", 14, 'MarkerIndices', 1:2:length(defl_errs))

xlabel("Time (sec)", "FontWeight","bold", "FontSize", 12)
ylabel("Tracking Error", "FontWeight","bold", "FontSize", 12)
title("Tracking Error of Controller-1 in Following Reference Model A",...
      "FontWeight","bold", "FontSize", 12);
legend("Deflection Error (mm)", "Oscillation Error (mm/s)")
grid on
grid minor
hold off


