%% Dynamic Autorouting - Komsun Tamanakijprasart (2023)
% For Static Results
clc, clear, close all

% ___________________Simulation Set-up Parameters__________________________
fontSize = 20;
saveVid = 0;
animation = 0;              % Figure(69)m 1: see the simulation
showDisp = 1;
tsim = uint16(400);          % [s] simulation time for the path 

dt = 0.1;                    % [s] simulation time step
dt_traj = 1;
rtsim = 50 / dt_traj;                   % [s] (50) time for the whole scenario 
simMode = uint8(2);          % 1: by time, 2: by target distance
targetThresh = 1;          % [m] allowed error for final target distance 
multiTarget = uint8(0);      % 1: multi-target 0: single-target
scene = 3;      % Scenario selection
                % 0) NO object 1) 1 object, 2) 2 objects 
                % 3) 3 objects 4) 3 complex objects
                % 7) non-urban 12) urban environment

% ___________________Features Control Parameters___________________________
useOptimizer = 0; % 0:Off  1:Global optimized  2: Local optimized
delta_g = 10;            % [m]  minimum allowed gap distance
k = 0;   % Higher(1000) = more effect from weather
           % Lower(~0.01) = less effect  0 = no weather effect

env = "static";    % "static" "dynamic"

% ______________________IFDS Tuning Parameters_____________________________
sf    = uint8(0);   % Shape-following demand (1=on, 0=off)
rho0  = 2.5;          % Repulsive parameter (rho >= 0)
sigma0 = 0.01;      % Tangential parameter 

% Good: rho0 = 2, simga0 = 0.01
% The algorihtm still doesnt work for overlapped objects

% _________________Constraint Matrix Tuning Paramters______________________
B_U = 0.9;   % [B_L < B_U <= 1]
B_L = 0;   % [0 < B_L < B_U]

% Good: k=1 | B_u=0.7 | B_L = 0
%     : k=100 | B_u=1 | B_L = 0.5

% _______________________CCA Tuning Parameters_____________________________
ccaTuning = 5;
switch ccaTuning
    case 1
        kappa =  10 ;              % Gain
        delta =  2;                % Carrot Distance
        kd = 0;
    case 2
        kappa = 20;
        delta = 5;
        kd = 0;
    case 3
        kappa = 10;
        delta = 10;
        kd = 0.1;
    case 4
        kappa = 100;
        delta = 1;
        kd = 0;
    case 5
        kappa = 50;
        delta = 20;
        kd = 0;
end

tuning = [kappa, delta, kd];

% _______________________ UAV Parameters _________________________________
C  = 10;             % [m/s] UAV cruising speed (30)
% Starting location
Xini = 0;
Yini = 0;
Zini = 0;

% Target Destination
Xfinal = 200;
Yfinal = 0;
% Zfinal = 10;
Zfinal = 50;

% UAV's Initial State
x_i = 0;
y_i = -20;
% y_i = 0;
z_i = 5;
psi_i = 0;          % [rad] Initial Yaw angle
gamma_i = 0;        % [rad] Initial Pitch angle


x_guess = [rho0; sigma0];

% Initialize constraint matrix (separate file)
initialize_constraint_matrix


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
Param.Rg = delta_g;
Param.rho0_initial = rho0;
Param.sigma0_initial = sigma0;
Param.Xini = Xini;
Param.Yini = Yini;
Param.Zini = Zini;
Param.Xfinal = Xfinal;
Param.Yfinal = Yfinal;
Param.Zfinal = Zfinal;
Param.useOptimizer = useOptimizer;
Param.k = k;
Param.B_U = B_U;
Param.B_L = B_L;


% Structure Pre-allocation for each scene
switch scene
    case 0, numObj = 1;
    case 1, numObj = 1;
    case 2, numObj = 2;
    case 3, numObj = 3;
    case 4, numObj = 3;
    case 5, numObj = 3;
    case 7, numObj = 7;
    case 12, numObj = 12;
    case 41, numObj = 3;
    case 42, numObj = 4;
    case 44, numObj = 7;
    case 69, numObj = 4;
    case 6969, numObj = 3;
end
Param.numObj = numObj;
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
timer = zeros(1,rtsim);

% Pre-allocate waypoints and path
Wp = zeros(3, tsim+1);
Paths = cell(numLine,rtsim);

%% ====================== Main Path-Planning Program ======================

traj = cell(1,rtsim);
traj{1} = [x_i, y_i, z_i];
errn = cell(1,rtsim);

for rt = 1:rtsim
    err = [];
    tic
    if norm([x_i y_i z_i] - [Xfinal Yfinal Zfinal]) < targetThresh  % [m]
        disp("Target destination reached at t = " + num2str(rt) + " s")
        traj = traj(~cellfun('isempty',traj));
        break
    end
                
    if scene == 41 || scene == 42 || (k ~= 0 && env == "dynamic") || scene == 44
        Wp(:,1) = [x_i; y_i; z_i];
    else
        Wp(:,1) = [Xini; Yini; Zini];  % can change this to current uav pos
    end

    if isempty(traj{rt})
        traj{rt} = traj{rt-1}(:,end);
    end

    for L = 1:numLine
        
        loc_final = destin(L,:)';
        %------------Global Path Optimization-------------
        if useOptimizer == 1
           [rho0, sigma0] = path_optimizing(loc_final, rt, Wp, Paths, Param, Object, WMCell{rt}, dwdxCell{rt}, dwdyCell{rt});
        end
        %------------------------------------------------
        
        % Compute the IFDS Algorithm
        if env == "dynamic"
            [Paths, Object, ~, foundPath] = IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, L, Object, WMCell{rt}, dwdxCell{rt}, dwdyCell{rt});
        elseif env == "static"
            [Paths, Object, ~, foundPath] = IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, L, Object, WMCell{15}, dwdxCell{15}, dwdyCell{15});
        end
%         timer(L) = toc;

    end

    if foundPath ~= 1 || isempty(Paths{rt}) || size(Paths{rt},2)==1
        disp("CAUTION : Path not found at t = " +num2str(rt) + " s")
        disp("*UAV is standing by*")
        continue
    end

    % Compute Path Following Algorithm
    trajectory = zeros(3, length(Paths{rt}));
    trajectory(:,1) = [x_i; y_i; z_i];

    i = 1;
    dtcum = 0;
    
    for j = 1:length(Paths{rt})-1
        if dtcum >= dt_traj
            break
        end 
        Wi = Paths{rt}(:,j);
        Wf = Paths{rt}(:,j+1);
    
        path_vect = Wf - Wi;
        a = path_vect(1);
        b = path_vect(2);
        c = path_vect(3);
        
        % Check if the waypoint is ahead of current position
        if a*(x_i - Wf(1)) + b*(y_i - Wf(2)) + c*(z_i - Wf(3)) < 0
            
            err = [err, norm([x_i-Wf(1), y_i-Wf(2), z_i-Wf(3)])];
            [x, y, z, psi, gamma, timeSpent] = CCA3D_straight(Wi, Wf, x_i, y_i, z_i, psi_i, gamma_i, C, tuning);
            x_i = x(end);
            y_i = y(end);
            z_i = z(end);
            psi_i = psi(end);
            gamma_i = gamma(end);
            dtcum = dtcum + timeSpent;
          
            trajectory(:,i+1) = [x y z]';
            i = i+1;
        else
%             disp("skip waypoint #" + num2str(j)) 
        end   
    end
    errn{rt} = err;
    trajectory = trajectory(:,1:i);   % remove extra element
    traj{rt} = trajectory;
    timer(rt) = toc;
    disp("Computed time = " + num2str((timer(rt))) + " s")

end

% timer(timer==0)=[];
disp("Average computed time = " + num2str(mean(timer(timer~=0))) + " s")


%% CCA3D Error Analysis - For static path only!!
errn = errn(~cellfun('isempty',errn));
figure(91)
for j = 1:length(errn)
    if mod(j,2) == 0
        styl = 'o-k';
    else
        styl = 'o-b';
    end
    plot(linspace(j-1,j,size(errn{j},2)), errn{j}, styl, 'LineWidth', 1), hold on
end
grid on, grid minor
%% =======================Plotting Results=================================

for rt = 1:size(traj,2)
    
    % Plotting the path
    figure(70)
    set(gcf, 'Position', get(0, 'Screensize'));
    subplot(1,2,1)

    quiver3(traj{rt}(1,1), traj{rt}(2,1), traj{rt}(3,1),...
        traj{rt}(1,end)-traj{rt}(1,1), traj{rt}(2,end)-traj{rt}(2,1),...
        traj{rt}(3,end)-traj{rt}(3,1), 'ok','filled', 'LineWidth', 1.5, 'MaxHeadSize',100,'AutoScaleFactor', 2,...
        'Alignment','tail', 'MarkerSize', 10, 'MarkerFaceColor','w','ShowArrowHead','on')

    hold on, grid on, axis equal

    if ~isempty(Paths{rt})
        PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
    end
    if rt>1
        prevTraj = [traj{1:rt-1}];
        plot3(prevTraj(1,:), prevTraj(2,:), prevTraj(3,:), 'k', 'LineWidth', 1.2) 
    end

    scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
    
    if k~=0
        set(gca, 'YDir', 'normal')
        colormap turbo
        if env == "dynamic"
            contourf(1:200,-100:99,weatherMatMod(:,:,rt), 30)
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,rt), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
        elseif env == "static"
            contourf(1:200,-100:99,weatherMatMod(:,:,15), 30)
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,15), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
        end
    end
    
    xlim([0 200]), ylim([-100 100]), zlim([0 100])
    title(num2str(rt*dt_traj,'time = %4.2f s')) 
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    set(gca, 'LineWidth', 2, 'FontSize', fontSize-6)
%     view(0,90)
    hold off

    subplot(1,2,2)

    quiver3(traj{rt}(1,1), traj{rt}(2,1), traj{rt}(3,1),...
        traj{rt}(1,end)-traj{rt}(1,1), traj{rt}(2,end)-traj{rt}(2,1),...
        traj{rt}(3,end)-traj{rt}(3,1), 'ok','filled', 'LineWidth', 1.5, 'MaxHeadSize',100,'AutoScaleFactor', 2,...
        'Alignment','tail', 'MarkerSize', 10, 'MarkerFaceColor','w','ShowArrowHead','on')

    hold on, grid on, axis equal

    if ~isempty(Paths{rt})
        PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
    end
    if rt>1
        prevTraj = [traj{1:rt-1}];
        plot3(prevTraj(1,:), prevTraj(2,:), prevTraj(3,:), 'k', 'LineWidth', 1.2)
        
    end

    scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
    
    if k~=0
        set(gca, 'YDir', 'normal')
        colormap turbo
        
        if env == "dynamic"
            contourf(1:200,-100:99,weatherMatMod(:,:,rt), 30)
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,rt), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
        elseif env == "static"
            contourf(1:200,-100:99,weatherMatMod(:,:,15), 30)
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,15), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
        end
        
        clabel(C2,h2,'FontSize',15,'Color','w')
    end

    
    xlim([0 200])
    ylim([-100 100])
    zlim([0 100])
    
    title(num2str(rt*dt_traj,'time = %4.2f s')) 
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    set(gca, 'LineWidth', 2, 'FontSize', fontSize-6)
    view(0,90)
    hold off
%     pause(0.1)

end

syms X Y Z Gamma(X,Y,Z) Gamma_star(X,Y,Z) Gamma_prime(X,Y,Z)
syms omega(X,Y) wet(X,Y)

%%
figure(69)
if animation
    simulate = 1:size(traj,2);
else
    simulate = size(traj,2);
end
for rt = simulate
    if rt>2
        prevTraj = [traj{1:rt-1}];
    end

    figure(69)
    set(gcf, 'Position', get(0, 'Screensize'));
    subplot(7,2,[1 3 5 7])
    plotting_everything
    
    if k~=0
        hold on
        set(gca, 'YDir', 'normal')
 
        if env == "dynamic"
            contourf(1:200,-100:99,weatherMatMod(:,:,rt),30,'LineStyle', '-')
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,rt), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
            contourf(1:200,-100:99,weatherMatMod(:,:,rt), 30)
        elseif env == "static"
            contourf(1:200,-100:99,weatherMatMod(:,:,15),30,'LineStyle', '-')
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,15), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
        end
        hold off
    end
    % set(gca, "FontSize", 18)
    subplot(7,2,[2 4 6 8]);
    plotting_everything
    if k~=0
        hold on
        set(gca, 'YDir', 'normal')
        % colormap(flipud(bone))
        colormap turbo 
        if env == "dynamic"
            contourf(1:200,-100:99,weatherMatMod(:,:,rt),30,'LineStyle', '-')
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,rt), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
            contourf(1:200,-100:99,weatherMatMod(:,:,rt), 30)
        elseif env == "static"
            contourf(1:200,-100:99,weatherMatMod(:,:,15),30,'LineStyle', '-')
            [C2,h2] = contourf(1:200, -100:99, weatherMat(:,:,15), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
        end
        clabel(C2,h2,'FontSize',15,'Color','w')
        colorbar
        hold off
    end
    if ~animation
        if rt>1
            legend([pltDestin, pltPath, pltTraj], "Destination", "IFDS Path", "UAV Trajectory",'Position',[0.757 0.917 0.09 0.04])
            % legend([pltDestin, pltTraj], "Destination", "UAV Trajectory",'Position',[0.757 0.917 0.09 0.04])
        else
            legend([pltDestin, pltPath], "Destination", "IFDS Path",'Position',[0.757 0.917 0.09 0.04])
        end
    end

    view(0,90)
    % set(gca, "FontSize", 18)

    subplot(7,2,[9 11 13])
    plotting_everything
    view(90,0)
    % set(gca, "FontSize", 18)

    subplot(7,2,[10 12 14])
    plotting_everything
    view(0,0)
    % set(gca, "FontSize", 18)


    if k ~=0
    sgtitle([['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0) ', SF = ' num2str(sf),', \delta_g = ', num2str(delta_g), 'm, ', num2str(rt,'time = %4.1f s')]; ...
        "Constraint Matrix, k = " + num2str(k) +  ", B_U = " + num2str(B_U) + ", B_L = " + num2str(B_L)], 'FontSize', fontSize+2);
    else
    sgtitle(['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0) ', SF = ' num2str(sf),', \delta_g = ', num2str(delta_g), 'm, ', num2str(rt,'time = %4.1f s')],'FontSize', fontSize+2);
    end
    % Video saving
    if saveVid
        frm(rt) = getframe(gcf) ;
        drawnow
    end
end



% title(['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0)],...
%     'FontSize',26);
% subtitle(['SF = ' num2str(sf)], 'FontSize', 24)


if saveVid
    if k~=0
        text = "_weather";
    else
        text = "";
    end
    video_name = "Result_scene_" + num2str(scene) + text + ".avi";
    disp("Video saved: " + video_name);
    % create the video writer with 1 fps
    writerObj = VideoWriter(video_name);
%     writerObj.FrameRate = 30;
    writerObj.FrameRate = 2;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i=1:length(frm)
        % convert the image to a frame
        frame = frm(i) ;    
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
end

%% Plot Gamma Distribution

figure(96)
PlotGamma(Gamma, Gamma_star, X, Y, Z, fontSize - 8, weatherMatMod, k, B_U, B_L)


%% ------------------------------Function---------------------------------


function [rho0, sigma0] = path_optimizing(loc_final, rt, Wp, Paths, Param, Object, weatherMat, dwdx, dwdy)

    xg = [Param.rho0_initial; Param.sigma0_initial];

    lower_bound_rho = 0.05;  % <0.05 issue started to occur
    lower_bound_sigma = 0;
    
    upper_bound_rho = 2.5;
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
        [~, ~, totalLength] = ...
            IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, 1, Object, weatherMat, dwdx, dwdy);
   
    end
    
end

function PlotGamma(Gamma, Gamma_star, X, Y, Z, fontSize, weatherMat, k, B_U, B_L)
    xr = 0:1:200;
    yr = -100:1:100;
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


%     Gamma_numeric = Gamma_numeric +  1 - exp(omega * log(Gamma_numeric));
    
    % Calculate Gamma for each  pair plane
    Gamma_values_XY_og = Gamma_numeric(X_grid_xy, Y_grid_xy, zfixed);
    Gamma_values_XY = Gamma_numeric_mod(X_grid_xy, Y_grid_xy, zfixed);
    Gamma_values_YZ = Gamma_numeric_mod(xfixed, Y_grid_yz, Z_grid_yz);
    Gamma_values_XZ = Gamma_numeric_mod(X_grid_xz, yfixed, Z_grid_xz);
    
    % Define the number of countour levels
    num_levels = 30;
    max_Gamm = max([max(max(Gamma_values_XY)), max(max(Gamma_values_YZ)), max(max(Gamma_values_XZ))]);

    % Plot the Gamma distribution
    sp1 = subplot(2,3,1);
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
    set(gca, 'FontSize', fontSize+2)

    subplot(2,3,2)
    colormap(flipud(turbo))
    contourf(X_grid_xy, Y_grid_xy, Gamma_values_XY_og, num_levels), hold on
    [C1,h1] = contourf(X_grid_xy, Y_grid_xy, Gamma_values_XY_og, [1, 1], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    clabel(C1,h1,'FontSize',15,'Color','w')
    xlabel('X [m]'), ylabel('Y [m]')
    title("Original \Gamma - Top view (Z = " + num2str(zfixed) + ")");
    grid on, grid minor, axis equal tight, hold off
    colorbar
    set(gca, 'FontSize', fontSize+2)

    sp3 = subplot(2,3,3);
    set(gca, 'YDir', 'normal')
    contourf(1:200,1:200, weatherMat(:,:,1), num_levels); 
    hold on
    [C2,h2] = contourf(1:200, 1:200, weatherMat(:,:,1), [1, 1], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    clabel(C2,h2,'FontSize',15,'Color','w')
    xlabel('X [m]'), ylabel('Y [m]')
    title("Original Constraint Matrix");
    grid on, grid minor, axis equal tight, hold off
    set(gca, 'FontSize', fontSize+2)
    colormap(sp3,turbo)
    colorbar
   
    subplot(2,3,4)
    contourf(X_grid_xy, Y_grid_xy, Gamma_values_XY, num_levels), hold on
    [C3,h3] = contourf(X_grid_xy, Y_grid_xy, Gamma_values_XY, [0, 1.00001], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    colorbar
%     clim([0 max_Gamm])
    clabel(C3,h3,'FontSize',15,'Color','w')
    xlabel('X [m]'), ylabel('Y [m]')
    title("\Gamma_p - Top view (Z = " + num2str(zfixed) + ")");
    grid on, grid minor, axis equal tight, hold off
    set(gca, 'FontSize', fontSize+2)
    
    % Plot the Y-Z plane distribution
    subplot(2,3,5)
    contourf(Y_grid_yz, Z_grid_yz, Gamma_values_YZ, num_levels), hold on
    [C4,h4] = contourf(Y_grid_yz, Z_grid_yz, Gamma_values_YZ, [1, 1], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    colorbar
%     clim([0 max_Gamm])
    clabel(C4,h4,'FontSize',15,'Color','w')
    xlabel('Y [m]'), ylabel('Z [m]')
    title("\Gamma_p - Front view (X = " + num2str(xfixed) + ")");
    grid on, grid minor, axis equal tight, hold off
    set(gca, 'FontSize', fontSize+2, 'XDir', 'reverse')
    
    
    % Plot the X-Z plane distribution
    subplot(2,3,6)
    contourf(X_grid_xz, Z_grid_xz, Gamma_values_XZ, num_levels), hold on
    [C5,h5] = contourf(X_grid_xz, Z_grid_xz, Gamma_values_XZ, [1, 1], 'FaceAlpha',0,...
        'LineColor', 'w', 'LineWidth', 2);
    colorbar
%     clim([0 max_Gamm])
    clabel(C5,h5,'FontSize',15,'Color','w')
    xlabel('X [m]'), ylabel('Z [m]');
    title("\Gamma_p - Side view (Y = " + num2str(yfixed) +")");
    grid on, grid minor, axis equal tight, hold off
    set(gca, 'FontSize', fontSize+2)
    
    colormap(sp1, pink)
    sgt = sgtitle('Distribution of the boundary function');
    sgt.FontSize = fontSize + 10;

    function gam = Gamma_numeric_mod(X,Y,Z)
        gam = Gamma_numeric(X,Y,Z);
        
        if size(X,1) > 1 && size(Y,1) >1

            xid = round(X)+1;
            yid = round(Y)+100+1;
            
            xid(xid>200) = 200;
            yid(yid>200) = 200;
            omega = zeros(size(xid));
            
            for i = 1:size(xid,1)
                
                for j = 1:size(xid,2)
                    omega(i,j) = weatherMat(yid(i,j), xid(i,j), 1);
                end
            end

            gam = 1 + gam - exp(omega .* log(gam));
%             gam = gam - k* (exp( (B_L - omega)/(B_L - B_U) * log((gam-1)/k +1) ) -1);
             
        end

        
    end
end

% function [Gamma, Gamma_star] = PlotObject(Object, Rg, rt, rtsim, X, Y, Z, Gamma, Gamma_star)
%     for j = 1:size(Object,2)
%         x0 = Object(j).origin(rt, 1);
%         y0 = Object(j).origin(rt, 2);
%         z0 = Object(j).origin(rt, 3);
%         a = Object(j).a;
%         b = Object(j).b;
%         c = Object(j).c;
%         p = Object(j).p;
%         q = Object(j).q;
%         r = Object(j).r;
% 
%         Rstar = Object(j).Rstar;
%     
%         Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
%         Gamma_star(X, Y, Z) = Gamma - ( (Rstar + Rg)/Rstar )^2 + 1;
% 
% %         if rtsim > 1
% %             fimplicit3(Gamma == 1,'EdgeColor','k','FaceAlpha',1,'MeshDensity',20), hold on
% %             fimplicit3(Gamma_star == 1, 'EdgeColor','k','FaceAlpha',0,'MeshDensity',20)
% %         else
%             fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',0.9,'MeshDensity',80), hold on
%             fimplicit3(Gamma_star == 1, 'EdgeColor','none','FaceAlpha',0.2,'MeshDensity',30)
% %         end
% %         colormap pink
% 
%         xlim([0 200])
%         ylim([-100 100])
%         zlim([0 100])
%     end
% 
% end

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
        plot3(Paths{1,rt}(1,:), Paths{1,rt}(2,:), Paths{1,rt}(3,:),'b--', 'LineWidth', 1.8)
        hold on
%         axis equal, grid on, grid minor
        scatter3(Xini, Yini, Zini, 'filled', 'r', 'xr', 'sizedata', 150)
        scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
    end

    xlim([0 200])
    ylim([-100 100])
    zlim([0 100])
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
%     hold off
end
