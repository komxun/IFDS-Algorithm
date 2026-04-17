clc, clear, close all

% Initial condition
% Set-up Parameters
tsim = uint16(400);          % [s] simulation time for the path 
rtsim = 1;                   % [s] (50) time for the whole scenario 
dt = 0.1;            % [s] simulation time step
C  = 30;             % [m/s] UAV cruising speed
targetThresh = 2.5;  % [m] allowed error for final target distance 
simMode = uint8(1);          % 1: by time, 2: by target distance
multiTarget = uint8(0);      % 1: multi-target 0: single-target
scene = uint8(2);       % Scenario selection
                        % 1) 1 cone, 2) 1 complex object
                        % 7) non-urban 12) urban environment

% Starting location
Xini = 0;
Yini = 0;
Zini = 0;

% Target Destination
Xfinal = 200;
Yfinal = 0;
Zfinal = 50;

% Tuning Parameters
sf    = uint8(1);         % Shape-following demand (1=on, 0=off)
rho0  = 1.3;        % Repulsive parameter (rho >= 0)
sigma0 = 0.01;    % Tangential parameter 

%----------- Note -------------
% Good: rho0 = 2, simga0 = 0.01
% Algorihtm stills doesnt work for overlapped objects
%------------------------------

% Save to table Param
Param = table;
Param.tsim = tsim;
Param.rtsim = rtsim;
Param.dt = dt;
Param.C = C;
Param.targetThresh = targetThresh;
Param.simMode = simMode;
Param.multiTarget = multiTarget;
Param.scene = scene;
Param.sf = sf;
Param.Xini = Xini;
Param.Yini = Yini;
Param.Zini = Zini;
Param.Xfinal = Xfinal;
Param.Yfinal = Yfinal;
Param.Zfinal = Zfinal;

% Initial guess for decision variables rho and gamma
initial_rho0 = 1;
initial_sigma0 = 0.01;
x0 = [initial_rho0; initial_sigma0];

lower_bound_rho = 0;
lower_bound_sigma = 0;

upper_bound_rho = 2;
upper_bound_sigma = 1;

% Set up the optimization problem
problem.objective = @(x) path_dist_objective_v2(x(1), x(2), Param);
problem.x0 = x0;
problem.Aineq = [];
problem.bineq = [];
problem.Aeq = [];
problem.beq = [];
problem.lb = [lower_bound_rho; lower_bound_sigma];
problem.ub = [upper_bound_rho; upper_bound_sigma];
problem.nonlcon = [];
problem.solver = 'fmincon';  % specify the solver
problem.options = optimoptions('fmincon', ...
    'Algorithm', 'interior-point', ...   % option2: sqp
    'Display', 'iter');


% Call fmincon
[xOpt, fval, exitflag, output] = fmincon(problem);
path_dist_objective_v2(rho0, sigma0, Param)
%% Display the results
disp('Optimized decision variables (rho0, sigma0):');
disp(xOpt);
disp('Optimized path distance (m):');
disp(fval);
disp("Original path distance = " + path_dist_objective_v2(x0(1),x0(2), Param) + " m")
