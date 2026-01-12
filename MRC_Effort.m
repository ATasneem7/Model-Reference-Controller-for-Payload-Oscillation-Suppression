function u = MRC_Effort(xd1, xd2, v, x1, x2)

% declaring these variables global so they can be accessed here here
global Ap  A  P;

x  = [x1; x2];                  % Plant State vectors
xd = [xd1; xd2];                % Desired State vectors
e = xd - x;                     % Tracking error


  u = v - (3*sqrt(109)*x2)/50;    % Control Law for zeta = 0.1
% u = v-(3*sqrt(109)*x2)/10;      % Control Law for zeta = 0.5
% u = v-(12*sqrt(109)*x2)/25;     % Control Law for zeta = 0.8  
return;
