clc, clear, close all

% Set-up Parameters
fontSize = 20;
showDisp = 1;
tsim = uint16(400);          % [s] simulation time for the path 
rtsim = 1;                   % [s] (50) time for the whole scenario 
dt = 0.1;            % [s] simulation time step
C  = 30;             % [m/s] UAV cruising speed
targetThresh = 2.5;  % [m] allowed error for final target distance 
simMode = uint8(1);          % 1: by time, 2: by target distance
multiTarget = uint8(0);      % 1: multi-target 0: single-target
scene = uint8(0);       % Scenario selection
                        % 0) 1 cone, 2) 1 complex object
                        % 7) non-urban 12) urban environment

useOptimizer = 0; % 0:Off  1:Global optimized  2: Local optimized

% Starting location
Xini = 0;
Yini = 0;
Zini = 0;

% Target Destination
Xfinal = 200;
Yfinal = 0;
Zfinal = 10;

% Tuning Parameters
sf    = uint8(0);   % Shape-following demand (1=on, 0=off)
rho0  = 1;        % Repulsive parameter (rho >= 0)
sigma0 = 0.01;      % Tangential parameter 
Rg = 0;            % [m]  minimum allowed gap distance

x_guess = [rho0; sigma0];

%----------- Note -------------
% Good: rho0 = 2, simga0 = 0.01
% The algorihtm still doesnt work for overlapped objects
%------------------------------

% Save to table Param
Param = table;
Param.showDisp = showDisp;
Param.tsim = tsim;
Param.rtsim = rtsim;
Param.dt = dt;
Param.C = C;
Param.targetThresh = targetThresh;
Param.simMode = simMode;
Param.multiTarget = multiTarget;
Param.scene = scene;
Param.sf = sf;
Param.Rg = Rg;
Param.rho0_initial = rho0;
Param.sigma0_initial = sigma0;
Param.Xini = Xini;
Param.Yini = Yini;
Param.Zini = Zini;
Param.Xfinal = Xfinal;
Param.Yfinal = Yfinal;
Param.Zfinal = Zfinal;
Param.useOptimizer = useOptimizer;

% Structure Pre-allocation for each scene
switch scene
    case 0, numObj = 1;
    case 1, numObj = 1;
    case 2, numObj = 2;
    case 3, numObj = 3;
    case 7, numObj = 7;
    case 12, numObj = 12;
    case 41, numObj = 1;
    case 42, numObj = 4;
    case 69, numObj = 4;
end

Object(numObj) = struct('origin',zeros(rtsim,3),'Gamma',0,'n',[],'t',[],...
    'a',0,'b',0,'c',0,'p',0,'q',0,'r',0,'Rstar',0);

disp(['Number of object: ' num2str(size(Object,2))])
if sf == 0, disp("Shape-following: Off") 
else, disp("Shape-following: On")
end

switch useOptimizer
    case 0,  disp("Path optimization: Off")
    case 1,  disp("Path optimization: Global");
    case 2,  disp("Path optimization: Local")
end


%% Original Fluid

if multiTarget
    destin = [200 0   20;
              200 20  20;
              200 -20 20;
              200 20  30;
              200 -20 30;
              200 0  30;
              200 0  40;
              200 20 40;
              200 -20 40;];
else
    destin = [Xfinal Yfinal Zfinal];
end

numLine = size(destin,1);
disp("Generating paths for " + num2str(numLine) + " destinations . . .")
disp("*Timer started*")
timer = zeros(1,numLine);

% Pre-allocate waypoints and path
Wp = zeros(3, tsim+1);
Paths = cell(numLine,rtsim);

%% ====================== Main Path-Planning Program ======================

rho0List = [0.5 1 2.5 5 10] ; 
sigma0List = [0.01];
figure(70)


leg = [];
for j = 1:length(rho0List)
    rho0 = rho0List(j);
%     Param.sigma0_initial = sigma0;
    for k = 1:length(sigma0List)
        sigma0 = sigma0List(k);

        leg = [leg, "\rho_0 = " + num2str(rho0)];

        for rt = 1:rtsim
            tic
            Wp(:,1,rt) = [Xini; Yini; Zini];  % can change this to current uav pos
        
            for L = 1:numLine
                
                loc_final = destin(L,:)';
                
                %------------Global Path Optimization-------------
                if useOptimizer == 1
                   [rho0, sigma0] = path_optimizing(loc_final, rt, Wp, Paths, Param, Object);
                end
                %------------------------------------------------
                
                % Compute the IFDS Algorithm
                [Paths, Object, ~] = IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, L, Object);
                timer(L) = toc;
        
            end
        
            disp("Average computed time = " + num2str(mean(timer)) + " s")
            % Plotting the path
            PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget), hold on
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
                
        end
    end
end

syms X Y Z Gamma(X,Y,Z) Gamma_star(X,Y,Z) 
PlotObject(Object, Rg, 1, rtsim, X, Y, Z, Gamma, Gamma_star);
legend([leg, "", ""])
title("IFDS, \sigma_0 = " + num2str(sigma0))

set(gca, "FontSize", 25, 'LineWidth', 1.5)



% =========================================================================
%% Plotting Results
animation = 1;




% figure(69)
% % set(gcf, 'Position', get(0, 'Screensize'));
% for rt = 1:rtsim
%     figure(69)
%     subplot(2,2,1)
%     PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
%     [Gamma, Gamma_star] = PlotObject(Object, Rg, rt, rtsim, X, Y, Z, Gamma, Gamma_star);
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); camlight
%     grid minor
%     set(gca, 'LineWidth', 2)
%     hold off
% 
%     subplot(2,2,2);
%     PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
%     [Gamma, Gamma_star] = PlotObject(Object, Rg, rt, rtsim, X, Y, Z, Gamma, Gamma_star);
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); camlight
%     grid minor
%     set(gca, 'LineWidth', 2)
%     view(0,90)
%     hold off
% 
%     subplot(2,2,3)
%     PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
%     [Gamma, Gamma_star] = PlotObject(Object, Rg, rt, rtsim, X, Y, Z, Gamma, Gamma_star);
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); camlight
%     grid minor
%     set(gca, 'LineWidth', 2)
%     view(90,0)
%     hold off
% 
%     subplot(2,2,4);
%     PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
%     [Gamma, Gamma_star] = PlotObject(Object, Rg, rt, rtsim, X, Y, Z, Gamma, Gamma_star);
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); camlight
%     grid minor
%     set(gca, 'LineWidth', 2)
%     view(0,0)
%     hold off
% 
%     sgtitle(['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0) ', SF = ' ...
%         num2str(sf),', r_g = ', num2str(Rg), 'm, ', num2str(rt,'time = %4.1f s')], 'Fontsize', fontSize+2);
% end

% title(['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0)],...
%     'FontSize',26);
% subtitle(['SF = ' num2str(sf)], 'FontSize', 24)
% set(gca,'FontSize', fontSize)



%% ------------------------------Function---------------------------------


function [rho0, sigma0] = path_optimizing(loc_final, rt, Wp, Paths, Param, Object)

    xg = [Param.rho0_initial; Param.sigma0_initial];

    lower_bound_rho = 0.05;  % <0.05 issue started to occur
    lower_bound_sigma = 0;
    
    upper_bound_rho = 2;
    upper_bound_sigma = 1;
    
    % Set up the optimization problem
    problem.objective = @(x) PathDistObjective(x(1), x(2));
    problem.x0 = xg;
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
        'Display', 'off');
    
    % Call fmincon
    [xOpt, fval, exitflag, output] = fmincon(problem);
    disp(output)
    rho0 = xOpt(1);
    sigma0 = xOpt(2);

    function totalLength = PathDistObjective(rho0, sigma0)
        Param.showDisp = 0;
        Param.useOptimizer = 0;
        [~, ~, totalLength] = IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, 1, Object);
   
    end
    
end

function PlotGamma(Gamma, Gamma_star, X, Y, Z, fontSize)
    xr = 0:200;
    yr = -100:100;
%     zr = 0:100;
    zr = 0:200;
    
    % fixed plane
    xfixed = 100;
    yfixed = 0;
    zfixed = 0;
    
    % limits of the rectangular plane
    x_plane_limits = [min(xr), max(xr)];
    y_plane_limits = [min(yr), max(yr)];
    z_plane_limits = [min(zr), max(zr)];
    
    xy_vertices = [x_plane_limits(1), y_plane_limits(1), zfixed;
                   x_plane_limits(1), y_plane_limits(2), zfixed;
                   x_plane_limits(2), y_plane_limits(2), zfixed;
                   x_plane_limits(2), y_plane_limits(1), zfixed];
    yz_vertices = [xfixed, y_plane_limits(1), z_plane_limits(1);
                   xfixed, y_plane_limits(2), z_plane_limits(1);
                   xfixed, y_plane_limits(2), z_plane_limits(2);
                   xfixed, y_plane_limits(1), z_plane_limits(2)];
    xz_vertices = [x_plane_limits(1), yfixed, z_plane_limits(1);
                   x_plane_limits(1), yfixed, z_plane_limits(2);
                   x_plane_limits(2), yfixed, z_plane_limits(2);
                   x_plane_limits(2), yfixed, z_plane_limits(1)];
    faces = [1,2,3,4];

    % Create a grid of X and Y values for X-Y, Y-Z, and X-Z plane
    [X_grid_xy, Y_grid_xy] = meshgrid(xr, yr);
    [Y_grid_yz, Z_grid_yz] = meshgrid(yr, zr);
    [X_grid_xz, Z_grid_xz] = meshgrid(xr, zr);
    
    % Convert the symbolic function to a numerical function
    Gamma_numeric = matlabFunction(Gamma, 'Vars', [X, Y, Z]);

    % Calculate Gamma for each  pair plane
    Gamma_values_XY = Gamma_numeric(X_grid_xy, Y_grid_xy, zfixed);
    Gamma_values_YZ = Gamma_numeric(xfixed, Y_grid_yz, Z_grid_yz);
    Gamma_values_XZ = Gamma_numeric(X_grid_xz, yfixed, Z_grid_xz);
    
    % Define the number of countour levels
    num_levels = 30;
    max_Gamm = max([max(max(Gamma_values_XY)), max(max(Gamma_values_YZ)), max(max(Gamma_values_XZ))]);

    % Plot the Gamma distribution
    sp1 = subplot(2,2,1);
    fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1,'MeshDensity',80), hold on
    fimplicit3(Gamma_star == 1, 'EdgeColor','none','FaceAlpha',0.2,'MeshDensity',30)
    patch('Vertices', xy_vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    patch('Vertices', yz_vertices, 'Faces', faces, 'FaceColor', 'red', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    patch('Vertices', xz_vertices, 'Faces', faces, 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    axis equal
    xlim([0 200])
    ylim([-100 100])
    zlim([0 100])
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]')
    set(gca, 'FontSize', fontSize)

    subplot(2,2,2)
    colormap(flipud(jet))
    contourf(X_grid_xy, Y_grid_xy, Gamma_values_XY, num_levels), hold on
    [C3,h3] = contourf(X_grid_xy, Y_grid_xy, Gamma_values_XY, [1, 1], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    colorbar
    clim([0 max_Gamm])
    clabel(C3,h3,'FontSize',15,'Color','w')
    xlabel('X [m]'), ylabel('Y [m]')
    title("Top view (Z = " + num2str(zfixed) + ")");
    grid on, grid minor, axis equal tight, hold off
    set(gca, 'FontSize', fontSize)
    
    % Plot the Y-Z plane distribution
    subplot(2,2,3)
    contourf(Y_grid_yz, Z_grid_yz, Gamma_values_YZ, num_levels), hold on
    [C4,h4] = contourf(Y_grid_yz, Z_grid_yz, Gamma_values_YZ, [1, 1], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    colorbar
    clim([0 max_Gamm])
    clabel(C4,h4,'FontSize',15,'Color','w')
    xlabel('Y [m]'), ylabel('Z [m]')
    title("Front view (X = " + num2str(xfixed) + ")");
    grid on, grid minor, axis equal tight, hold off
    set(gca, 'FontSize', fontSize, 'XDir', 'reverse')
    
    % Plot the X-Z plane distribution
    subplot(2,2,4)
    contourf(X_grid_xz, Z_grid_xz, Gamma_values_XZ, num_levels), hold on
    [C5,h5] = contourf(X_grid_xz, Z_grid_xz, Gamma_values_XZ, [1, 1], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    colorbar
    clim([0 max_Gamm])
    clabel(C5,h5,'FontSize',15,'Color','w')
    xlabel('X [m]'), ylabel('Z [m]');
    title("Side view (Y = " + num2str(yfixed) +")");
    grid on, grid minor, axis equal tight, hold off
    set(gca, 'FontSize', fontSize)
    
    colormap(sp1, pink)
    sgt = sgtitle('Distribution of \Gamma(X, Y, Z)');
    sgt.FontSize = fontSize + 2;
end



function [Gamma, Gamma_star] = PlotObject(Object, Rg, rt, rtsim, X, Y, Z, Gamma, Gamma_star)
    for j = 1:size(Object,2)
        x0 = Object(j).origin(rt, 1);
        y0 = Object(j).origin(rt, 2);
        z0 = Object(j).origin(rt, 3);
        a = Object(j).a;
        b = Object(j).b;
        c = Object(j).c;
        p = Object(j).p;
        q = Object(j).q;
        r = Object(j).r;

        Rstar = Object(j).Rstar;
    
        Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        Gamma_star(X, Y, Z) = Gamma - ( (Rstar + Rg)/Rstar )^2 + 1;

        if rtsim > 1
            fimplicit3(Gamma == 1,'EdgeColor','k','FaceAlpha',1,'MeshDensity',20), hold on
            fimplicit3(Gamma_star == 1, 'EdgeColor','k','FaceAlpha',0,'MeshDensity',20)
        else
            fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1,'MeshDensity',80, 'FaceColor','w'), hold on
            fimplicit3(Gamma_star == 1, 'EdgeColor','none','FaceAlpha',0.2,'MeshDensity',30)
        end
%         colormap pink
        camlight
%         xlim([0 200])
        ylim([-60 60])
%         zlim([0,40])
    end

end

function PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
    if multiTarget
        plot3(Paths{1,rt}(1,:), Paths{1,rt}(2,:), Paths{1,rt}(3,:),'b', 'LineWidth', 1.5)
        hold on, grid on, grid minor, axis equal
        plot3(Paths{2,rt}(1,:), Paths{2,rt}(2,:), Paths{2,rt}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{3,rt}(1,:), Paths{3,rt}(2,:), Paths{3,rt}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{4,rt}(1,:), Paths{4,rt}(2,:), Paths{4,rt}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{5,rt}(1,:), Paths{5,rt}(2,:), Paths{5,rt}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{6,rt}(1,:), Paths{6,rt}(2,:), Paths{6,rt}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{7,rt}(1,:), Paths{7,rt}(2,:), Paths{7,rt}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{8,rt}(1,:), Paths{8,rt}(2,:), Paths{8,rt}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{9,rt}(1,:), Paths{9,rt}(2,:), Paths{9,rt}(3,:),'b', 'LineWidth', 1.5)
        scatter3(Xini, Yini, Zini, 'filled', 'r')
        scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(2,1),destin(2,2),destin(2,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(3,1),destin(3,2),destin(3,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(4,1),destin(4,2),destin(4,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(5,1),destin(5,2),destin(5,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(6,1),destin(6,2),destin(6,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(7,1),destin(7,2),destin(7,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(8,1),destin(8,2),destin(8,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(9,1),destin(9,2),destin(9,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
    else
        plot3(Paths{1,rt}(1,:), Paths{1,rt}(2,:), Paths{1,rt}(3,:), 'LineWidth', 2)
        hold on, grid on, grid minor, axis equal
%         scatter3(Xini, Yini, Zini, 'filled', 'r', 'xr', 'sizedata', 150)
%         scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150)
    end

%     xlim([0 200])
%     ylim([-100 100])
%     zlim([0 100])
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
%     hold off
end
