clc, clear, close all


% Set-up Parameters
tsim = uint16(400);          % [s] simulation time for the path 
rtsim = 1;                   % [s] (50) time for the whole scenario 
dt = single(0.1);            % [s] simulation time step
C  = single(30);             % [m/s] UAV cruising speed
targetThresh = single(2.5);  % [m] allowed error for final target distance 
simMode = uint8(1);          % 1: by time, 2: by target distance
multiTarget = uint8(0);      % 1: multi-target 0: single-target
scene = uint8(2);       % Scenario selection
                        % 1) 1 cone, 2) 1 complex object
                        % 7) non-urban 12) urban environment

% Starting location
Xini = single(0);
Yini = single(0);
Zini = single(0);

% Target Destination
Xfinal = single(200);
Yfinal = single(0);
Zfinal = single(50);

% Tuning Parameters
sf    = uint8(0);         % Shape-following demand (1=on, 0=off)
rho0  = single(1.3);        % Repulsive parameter (rho >= 0)
sigma0 = single(0.01);    % Tangential parameter 

%----------- Note -------------
% Good: rho0 = 2, simga0 = 0.01
% Algorihtm stills doesnt work for overlapped objects
%------------------------------

% Structure Pre-allocation
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

Object(numObj) = struct('origin',zeros(rtsim,3),'Gamma',0,'dGdx',0,'dGdy',0,'dGdz',0,'a',0,'b',0,'c',0,'p',0,'q',0,'r',0);


disp(['Number of object: ' num2str(size(Object,2))])
if sf == 0, disp("Shape-following: Off") 
else, disp("Shape-following: On")
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

% L = 1;



% Pre-allocate waypoints and path
Wp = zeros(3, tsim+1);
Paths = cell(numLine,rtsim);

for rt = 1:rtsim
    tic
    Wp(:,1,rt) = [Xini; Yini; Zini];  % can change this to current uav pos

for L = 1:numLine
%     disp("Calculating path for destination #" + num2str(L))
    xd = (destin(L,1));
    yd = (destin(L,2));
    zd = (destin(L,3));

    switch simMode
        case 1 % Simulate by time
%             disp("Simulating by time for " + num2str(tsim) + " s.")
            for t = 1:tsim
                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                Object = create_scene(scene, Object, xx, yy, zz, rt);

                [UBar, n, u] = calc_ubar(xx, yy, zz, xd, yd, zd, Object, rho0, sigma0, C);

                ntu =  n'*u;
       
                if norm([xx yy zz] - [xd yd zd]) < targetThresh
%                     disp('Target destination reached!')
                    Wp = Wp(:,1:t);
                    Paths{L,rt} = Wp;    % Save into cell array
                    break
                else
                    if ntu < 0 || sf == 1
                        Wp(:,t+1) = Wp(:,t) + UBar * dt;
            
                    elseif ntu >= 0 && sf == 0
                        Wp(:,t+1) = Wp(:,t) + u * dt;
                    end  
                end
            end

        case 2 % simulate by reaching distance
            disp("Simulating by distance until " + num2str(targetThresh) + " m error range")
            t = 1;
   
            while norm([Wp(1,t) Wp(2,t) Wp(3,t)] - double([xd yd zd])) > targetThresh

                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                if n(xx,yy,zz)'*u(xx,yy,zz) < 0 || sf == 1
        %             disp('case 1 activated')
                    Wp(:,t+1) = Wp(:,t) + double(UBar(Wp(1,t), Wp(2,t), Wp(3,t))) * dt;
        
                elseif n(xx,yy,zz)'*u(xx,yy,zz) >= 0 && sf == 0
        %             disp('case 2 activated')
                    Wp(:,t+1) = Wp(:,t) + double(u(Wp(1,t), Wp(2,t), Wp(3,t))) * dt;
                end  
                t = t+1;
            end
            % Removing extra space
            Wp = Wp(:,1:t,:); 
            disp('Target destination reached!')
    end

    timer(L) = toc;
end

disp("Average computed time = " + num2str(mean(timer)) + " s")
% Plotting the path
figure(70)
% set(gcf, 'Position', get(0, 'Screensize'));
plot3(Paths{1,rt}(1,:), Paths{1,rt}(2,:), Paths{1,rt}(3,:),'b', 'LineWidth', 1.5)
hold on, grid on, grid minor, axis equal
scatter3(Xini, Yini, Zini, 'filled', 'r')
scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr')
if multiTarget
    plot3(Paths{2,rt}(1,:), Paths{2,rt}(2,:), Paths{2,rt}(3,:),'b', 'LineWidth', 1.5)
    plot3(Paths{3,rt}(1,:), Paths{3,rt}(2,:), Paths{3,rt}(3,:),'b', 'LineWidth', 1.5)
    plot3(Paths{4,rt}(1,:), Paths{4,rt}(2,:), Paths{4,rt}(3,:),'b', 'LineWidth', 1.5)
    plot3(Paths{5,rt}(1,:), Paths{5,rt}(2,:), Paths{5,rt}(3,:),'b', 'LineWidth', 1.5)
    plot3(Paths{6,rt}(1,:), Paths{6,rt}(2,:), Paths{6,rt}(3,:),'b', 'LineWidth', 1.5)
    plot3(Paths{7,rt}(1,:), Paths{7,rt}(2,:), Paths{7,rt}(3,:),'b', 'LineWidth', 1.5)
    plot3(Paths{8,rt}(1,:), Paths{8,rt}(2,:), Paths{8,rt}(3,:),'b', 'LineWidth', 1.5)
    plot3(Paths{9,rt}(1,:), Paths{9,rt}(2,:), Paths{9,rt}(3,:),'b', 'LineWidth', 1.5)
    scatter3(destin(2,1),destin(2,2),destin(2,3), 'xr', 'sizedata', 100)
    scatter3(destin(3,1),destin(3,2),destin(3,3), 'xr', 'sizedata', 100)
    scatter3(destin(4,1),destin(4,2),destin(4,3), 'xr', 'sizedata', 100)
    scatter3(destin(5,1),destin(5,2),destin(5,3), 'xr', 'sizedata', 100)
    scatter3(destin(6,1),destin(6,2),destin(6,3), 'xr', 'sizedata', 100)
    scatter3(destin(7,1),destin(7,2),destin(7,3), 'xr', 'sizedata', 100)
    scatter3(destin(8,1),destin(8,2),destin(8,3), 'xr', 'sizedata', 100)
    scatter3(destin(9,1),destin(9,2),destin(9,3), 'xr', 'sizedata', 100)
end
title(num2str(rt,'time = %4.1f s'))
xlim([0 200])
ylim([-100 100])
zlim([0 100])
%     pause(0.05)
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
hold off
    
%     figure
%     subplot(3,1,1)
%     plot(Wp(1,:),'o-')
%     subplot(3,1,2)
%     plot(Wp(2,:),'o-')
%     subplot(3,1,3)
%     plot(Wp(3,:),'o-')
end

%% post-Calculation
% Calculate pairwise distances between waypoints
waypoints = Paths{1,1}';

% Calculate the differences between consecutive waypoints
differences = diff(waypoints);

% Calculate the squared distances for each coordinate
squaredDistances = sum(differences.^2, 2);

% Calculate the total path length
totalLength = sum(sqrt(squaredDistances));

% Display the total path length
fprintf('Total path length: %.2f m\n', totalLength);
fprintf('Total flight time: %.2f s\n', totalLength/C);

%% ----------------------- Plotting Results -------------------------------
animation = 1;


syms X Y Z Gamma(X,Y,Z)
figure(69)
% set(gcf, 'Position', get(0, 'Screensize'));
view(-43,52)
for rt = 1:rtsim
%     Oy = -80 + 2*single(rt);

%     Oy1 = -80 + 2*single(rt);
%     Oy2 = 80 - 5*single(rt);
    
    if multiTarget
        plot_multi(rt, Paths, Xini, Yini, Zini, destin)
    else
        plot_single(rt, Paths, Xini, Yini, Zini, destin)
    end

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
    
        Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
%         figure(69)
        fimplicit3(Gamma == 1,'EdgeColor','k','FaceAlpha',0,'MeshDensity',20)
%         colormap pink
        xlim([0 200])
        ylim([-100 100])
        zlim([0 100])
%         hold on, grid on, grid minor, axis equal
    end
    title(['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0) ', SF = ' num2str(sf)]);
    subtitle(num2str(rt,'time = %4.1f s'))
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    hold off
    
end



% title(['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0)],...
%     'FontSize',26);
% subtitle(['SF = ' num2str(sf)], 'FontSize', 24)
set(gca,'FontSize', 24)

%% ------------------------------Function---------------------------------
function plot_multi(rt, Paths, Xini, Yini, Zini, destin)
    figure(69) 
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
    scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(2,1),destin(2,2),destin(2,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(3,1),destin(3,2),destin(3,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(4,1),destin(4,2),destin(4,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(5,1),destin(5,2),destin(5,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(6,1),destin(6,2),destin(6,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(7,1),destin(7,2),destin(7,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(8,1),destin(8,2),destin(8,3), 'xr', 'xr', 'sizedata', 100)
    scatter3(destin(9,1),destin(9,2),destin(9,3), 'xr', 'xr', 'sizedata', 100)
    title(num2str(rt,'time = %4.1f s'))  
    xlim([0 200])
    ylim([-100 100])
    zlim([0 100])
end

function plot_single(rt, Paths, Xini, Yini, Zini, destin)
    figure(69) 
    plot3(Paths{1,rt}(1,:), Paths{1,rt}(2,:), Paths{1,rt}(3,:),'b', 'LineWidth', 1.5)
    hold on, grid on, grid minor, axis equal
    scatter3(Xini, Yini, Zini, 'filled', 'r', 'xr', 'sizedata', 100)
    scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 100)
    title(num2str(rt,'time = %4.1f s'))  
    xlim([0 200])
    ylim([-100 100])
    zlim([0 100])    
end


function [UBar, n, u]  = calc_ubar(X, Y, Z, xd, yd, zd, Obj, rho0, sigma0, C)

    dist = sqrt((X - xd)^2 + (Y - yd)^2 + (Z - zd)^2);

    u = -[C*(X - xd)/dist, C*(Y - yd)/dist, C*(Z - zd)/dist]';
    
    %% Components
    numObj = size(Obj,2);

    Mm = zeros(3);
    sum_w = 0;

    for j = 1:numObj
        dGdx = Obj(j).dGdx;
        dGdy = Obj(j).dGdy;
        dGdz = Obj(j).dGdz;
        % Reading Gamma for each object
        Gamma = Obj(j).Gamma;
        
        % Unit normal vector
        n = [dGdx; dGdy; dGdz]; 
    
        % Unit tangential vector
        t = [dGdy; -dGdx; 0];
    
        % Object Distance from UAV
        x0 = Obj(j).origin(1);
        y0 = Obj(j).origin(2);
        z0 = Obj(j).origin(3);
    
        % Calculate parameters
        [rho, sigma, dist_obj] = calc_params();
    
        % Modular Matrix (Perturbation Matrix
        M = eye(3) - n*n'/(abs(Gamma)^(1/rho)*(n')*n)...
        + t*n'/(abs(Gamma)^(1/sigma)*norm(t)*norm(n));  % tao is removed for now
      
        % Weight
        w = 1;
        for i = 1:numObj
            if i == j
                continue
            else
                w = w * (Obj(i).Gamma - 1)/...
                    ((Obj(j).Gamma - 1) + (Obj(i).Gamma - 1));
            end
        end
        sum_w = sum_w + w;

        % Saving to Field
        Obj(j).n = n;
        Obj(j).t = t;
        Obj(j).dist = dist_obj;
        Obj(j).rho = rho;
        Obj(j).sigma = sigma;
        Obj(j).M  = M;
        Obj(j).w = w;
    
    end

    for j = 1:numObj
        Obj(j).w_tilde = Obj(j).w/sum_w;
        Mm = Mm + Obj(j).w_tilde * Obj(j).M;
    end

    UBar = Mm*u; 

    function [rho, sigma, dist_obj] = calc_params()
        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);
        
        % Reactivity Parameter 
        rho = rho0 * exp(1 - 1/(dist_obj * dist));
        sigma = sigma0 * exp(1 - 1/(dist_obj * dist));
    end

end

function Obj = create_scene(num, Obj, X, Y, Z, rt)
    switch num
        case 0  % Single object
            Obj(1) = create_cone(100, 5, 0, 50, 80, Obj(1));
            
        case 1 % single(complex) object
            Obj(1) = create_cylinder(100, 5, 0, 25, 200, Obj(1));
            Obj(2) = create_pipe(60, 20, 60, 80, 5, Obj(2));
            Obj(3) = create_pipe(130, -30, 30, 100, 5, Obj(3));
    
        case 2 % 2 objects
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));
    
        case 3 % 3 objects
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));
            Obj(3) = create_pipe(168, 0, 0, 25, 80, Obj(3));
    
        case 12 % 12 objects
            Obj(1) = create_cylinder(100, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_pipe(140, 20, 0, 40,10, Obj(2));
            Obj(3) = create_pipe(20, 20, 0, 24, 40, Obj(3));
            Obj(4) = create_pipe(55, -20, 0, 28, 50, Obj(4));
            Obj(5) = create_sphere(53, -60, 0, 50, Obj(5));
            Obj(6) = create_pipe(150, -80, 0, 40, 50, Obj(6));
            Obj(7) = create_cone(100, -35, 0, 50,45, Obj(7));
            Obj(8) = create_cone(170, 2, 0, 20,50, Obj(8));
            Obj(9) = create_cone(60, 35, 0, 50,30, Obj(9));
            Obj(10) = create_cylinder(110, 70, 0, 60, 50, Obj(10));
            Obj(11) = create_pipe(170, 60, 0, 40, 27, Obj(11));
            Obj(12) = create_cone(150, -30, 0, 32, 45, Obj(12));
        case 7 % 7 objects
            Obj(1) = create_cone(60,8, 0, 70, 50, Obj(1));
            Obj(2) = create_cone(100,-24, 0, 89, 100, Obj(2));
            Obj(3) = create_cone(160,40, -4, 100, 30, Obj(3));
            Obj(4) = create_cone(100,100, -10, 150, 100, Obj(4));
            Obj(5) = create_cone(180,-70, -10, 150, 20, Obj(5));
            Obj(6) = create_cone(75,-75, -10, 150, 40, Obj(6));
            Obj(7) = create_cylinder(170, -6, 0, 34, 100, Obj(7));
        case 41
            Oy = -80 + 2*single(rt);
            Obj(1) = create_cylinder(100, Oy, 0, 50, 80, Obj(1));
        case 42
            Oy1 = 0 + 50*sin(0.2*single(rt));
            Oy2 = -20 - 20*sin(0.5*single(rt));
            Oz2 =  40 + 20*cos(0.5*single(rt));
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_cylinder(110, -10, 0, 25, 80,Obj(2));
            Obj(3) = create_cylinder(80, Oy1, 0, 20, 60, Obj(3));
            Obj(4) = create_sphere(160, Oy2, Oz2, 30, Obj(4));
    
        case 69 %pp
            Obj(1) = create_cylinder(100, 5, 0, 30, 80, Obj(1));
            Obj(2) = create_sphere(100, 30, 0, 40, Obj(2));
            Obj(3) = create_sphere(100, -20, 0, 40, Obj(3));
            Obj(4) = create_sphere(100, 5, 80, 30, Obj(4));
    end

    function Obj = create_sphere(x0, y0, z0, D, Obj)
        
        a = D/2;   b = D/2;   c = D/2;      % Object's axis length
        p = 1;     q = 1;     r = 1;        % Index parameters
       
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.dGdx = dGdx;
        Obj.dGdy = dGdy;
        Obj.dGdz = dGdz;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end

    function Obj = create_cylinder(x0, y0, z0, D, h, Obj)
    
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 5;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0]; 
        Obj.Gamma = Gamma;
        Obj.dGdx = dGdx;
        Obj.dGdy = dGdy;
        Obj.dGdz = dGdz;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end    
    end
    
    function Obj = create_cone(x0, y0, z0, D, h, Obj)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 0.5;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.dGdx = dGdx;
        Obj.dGdy = dGdy;
        Obj.dGdz = dGdz;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end    
    end
     
    function Obj = create_pipe(x0, y0, z0, D, h, Obj)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 2;     q = 2;     r = 2;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.dGdx = dGdx;
        Obj.dGdy = dGdy;
        Obj.dGdz = dGdz;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r; 
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end

    
end

