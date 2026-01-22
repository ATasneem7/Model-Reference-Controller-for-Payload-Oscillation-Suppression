function u = MRC_Effort(xd1, xd2, v, x1, x2)

% declaring these variables global so they can be accessed here here
global Ap  A  P;

x  = [x1; x2];                  % Plant State vectors
xd = [xd1; xd2];                % Desired State vectors
e = xd - x;                     % Tracking error


%%---Control Inputs of Scenario 1---%%

  u = v-(3*sqrt(109)*x2)/50;       % Control Law for zeta = 0.1
% u = v-(3*sqrt(109)*x2)/10;       % Control Law for zeta = 0.5
% u = v-(12*sqrt(109)*x2)/25;      % Control Law for zeta = 0.8  


%%---Control Inputs of Scenario 2---%%

% u = v+((327*x1)/50)-((3*sqrt(109)*x2)/50);     % u for zeta = 0.1, L = 0.6
% u = v+((327*x1)/50)-((3*sqrt(109)*x2)/10);     % u for zeta = 0.5, L = 0.6
% u = v+((327*x1)/50)-((12*sqrt(109)*x2)/25);    % u for zeta = 0.8, L = 0.6

% u = v+((981*x1)/400)-((3*sqrt(109)*x2)/50);    % u for zeta = 0.1, L = 0.8
% u = v+((981*x1)/400)-((3*sqrt(109)*x2)/10);    % u for zeta = 0.5, L = 0.8
% u = v+((981*x1)/400)-((12*sqrt(109)*x2)/25);   % u for zeta = 0.8, L = 0.8

% u = v-((2943*x1)/1300)-((3*sqrt(109)*x2)/50);    % u for zeta = 0.1, L = 1.3
% u = v-((2943*x1)/1300)-((3*sqrt(109)*x2)/10);    % u for zeta = 0.5, L = 1.3
% u = v-((2943*x1)/1300)-((12*sqrt(109)*x2)/25);   % u for zeta = 0.8, L = 1.3

% u = v-((327*x1)/100)-((3*sqrt(109)*x2)/50);      % u for zeta = 0.1, L = 1.5
% u = v-((327*x1)/100)-((3*sqrt(109)*x2)/10);      % u for zeta = 0.5, L = 1.5
% u = v-((327*x1)/100)-((12*sqrt(109)*x2)/25);     % u for zeta = 0.8, L = 1.5


%%---Control Inputs of Scenario 3---%%

% Case #01: Unrealistic Damping Ratio
% u = v-(3*sqrt(109)*x2)/5;                        % Control Law for zeta = 1.0
% u = v-(1939532120755487)*x2/281474976710656;     % Control Law for zeta = 1.1

% Case #02: Unrealistic Model Frequency
% u = v-(20191*x1)/15625-(833*x2)/250;        % Control Law for w_model = 3.332 rad/s
% u = v-(41641*x1)/15625-(883*x2)/250;        % Control Law for w_model = 3.532 rad/s
return;
