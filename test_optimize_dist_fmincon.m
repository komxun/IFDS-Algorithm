clc, clear, close all

% Initial guess for decision variables rho and gamma
initial_rho0 = 1;
initial_sigma0 = 0.01;
x0 = [initial_rho0; initial_sigma0];

lower_bound_rho = 0;
lower_bound_sigma = 0;

upper_bound_rho = 2;
upper_bound_sigma = 1;

% Set up the optimization problem
problem.objective = @(x) path_dist_objective_v2(x(1), x(2));
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

%% Display the results
disp('Optimized decision variables (rho0, sigma0):');
disp(xOpt);
disp('Optimized path distance (m):');
disp(fval);
disp("Original path distance = " + path_dist_objective_v2(x0(1),x0(2)) + " m")
