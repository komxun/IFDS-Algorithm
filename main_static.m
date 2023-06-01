clc, clear, close all


% Set-up Parameters
tsim = uint16(400);           % [s] simulation total time 
dt = single(0.1);            % [s] simulation time step
C  = single(30);             % [m/s] UAV cruising speed
targetThresh = single(2.5);  % [m] allowed error for final target distance 
simMode = uint8(1);          % 1: by time, 2: by target distance
multiTarget = uint8(0);      % 1: multi-target 0: single-target]
scene = uint8(0);       % Scenario selection
                        % 1) 1 cone, 2) 1 complex object
                        % 7) non-urban 12) urban environment

% Starting location
Xini = single(0);
Yini = single(0);
Zini = single(0);

% Target Destination
Xfinal = single(200);
Yfinal = single(0);
Zfinal = single(10);

% Tuning Parameters
sf    = uint8(0);         % Shape-following demand (1=on, 0=off)
rho0  = single(2);        % Repulsive parameter (rho >= 0)
sigma0 = single(0.01);    % Tangential parameter 

%----------- Note -------------
% Good: rho0 = 2, simga0 = 0.01
% Algorihtm stills doesnt work for overlapped objects
%------------------------------

% Structure Pre-allocation
switch scene
    case 0, Object(1) = struct('origin',[],'Gamma',[],'dGdx',[],'dGdy',[],'dGdz',[],'a',[],'b',[],'c',[],'p',[],'q',[],'r',[]);
    case 1, Object(1) = struct('origin',[],'Gamma',[],'dGdx',[],'dGdy',[],'dGdz',[],'a',[],'b',[],'c',[],'p',[],'q',[],'r',[]);
    case 2, Object(2) = struct('origin',[],'Gamma',[],'dGdx',[],'dGdy',[],'dGdz',[],'a',[],'b',[],'c',[],'p',[],'q',[],'r',[]);
    case 3, Object(2) = struct('origin',[],'Gamma',[],'dGdx',[],'dGdy',[],'dGdz',[],'a',[],'b',[],'c',[],'p',[],'q',[],'r',[]);
    case 7, Object(7) = struct('origin',[],'Gamma',[],'dGdx',[],'dGdy',[],'dGdz',[],'a',[],'b',[],'c',[],'p',[],'q',[],'r',[]);
    case 12, Object(12) = struct('origin',[],'Gamma',[],'dGdx',[],'dGdy',[],'dGdz',[],'a',[],'b',[],'c',[],'p',[],'q',[],'r',[]);
    case 69, Object(4) = struct('origin',[],'Gamma',[],'dGdx',[],'dGdy',[],'dGdz',[],'a',[],'b',[],'c',[],'p',[],'q',[],'r',[]);
end

disp(['Number of object: ' num2str(size(Object,2))])
if sf == 0, disp("Shape-following: Off") 
else, disp("Shape-following: On")
end

%% Original Fluid

if multiTarget
    destin = [200 0   0;
              200 20  0;
              200 -20 0;
              200 20  10;
              200 -20  10;
              200 0  10;
              200 0  20;
              200 20 20;
              200 -20 20;];
else
    destin = single([Xfinal Yfinal Zfinal]);
end

numLine = size(destin,1);
disp("Generating paths for " + num2str(numLine) + " destinations . . .")
disp("*Timer started*")
tic
for L = 1:numLine
    
    disp("Calculating path for destination #" + num2str(L))
    xd = (destin(L,1));
    yd = (destin(L,2));
    zd = (destin(L,3));

        
    %% Path for streamline
   
    Wp = single(zeros(3,tsim+1));
    Wp(:,1) = [Xini; Yini; Zini];

    switch simMode
        case 1 % Simulate by time
            disp("Simulating by time for " + num2str(tsim) + " s.")
            for j = 1:tsim
            
                xx = Wp(1,j);
                yy = Wp(2,j);
                zz = Wp(3,j);

                Object = create_scene(scene, Object, xx, yy, zz);

                [UBar, n, u] = calc_ubar(xx, yy, zz, xd, yd, zd, Object, rho0, sigma0, C);

                ntu =  n'*u;
       
                if norm([xx yy zz] - [xd yd zd]) < targetThresh
                    disp('Target destination reached!')
                    Wp = Wp(:,1:j);
                    break
                else
                    if ntu < 0 || sf == 1
                        Wp(:,j+1) = Wp(:,j) + UBar * dt;
            
                    elseif ntu >= 0 && sf == 0
                        Wp(:,j+1) = Wp(:,j) + u * dt;
                    end  
                end
            end

        case 2 % simulate by reaching distance
            disp("Simulating by distance until " + num2str(targetThresh) + " m error range")
            j = 1;
   
            while norm([Wp(1,j) Wp(2,j) Wp(3,j)] - double([xd yd zd])) > targetThresh

                xx = Wp(1,j);
                yy = Wp(2,j);
                zz = Wp(3,j);

                if n(xx,yy,zz)'*u(xx,yy,zz) < 0 || sf == 1
        %             disp('case 1 activated')
                    Wp(:,j+1) = Wp(:,j) + double(UBar(Wp(1,j), Wp(2,j), Wp(3,j))) * dt;
        
                elseif n(xx,yy,zz)'*u(xx,yy,zz) >= 0 && sf == 0
        %             disp('case 2 activated')
                    Wp(:,j+1) = Wp(:,j) + double(u(Wp(1,j), Wp(2,j), Wp(3,j))) * dt;
                end  
                j = j+1;
            end
            % Removing extra space
            Wp = Wp(:,1:j); 
            disp('Target destination reached!')
    end

    toc
    %% Plotting the path
    figure(69)   
    plot3(Wp(1,:), Wp(2,:), Wp(3,:),'b', 'LineWidth', 1.5)
    hold on, grid on, grid minor, axis equal
    
%     scatter3(Wp(1,1), Wp(2,1), Wp(3,1), 'filled', 'r')
    
    scatter3(xd, yd, zd, 200, 'xr')
    
%     figure
%     subplot(3,1,1)
%     plot(Wp(1,:),'o-')
%     subplot(3,1,2)
%     plot(Wp(2,:),'o-')
%     subplot(3,1,3)
%     plot(Wp(3,:),'o-')

end
%%----------------------- Plotting Results -------------------------------
syms X Y Z Gamma(X,Y,Z)

for j = 1:size(Object,2)
    x0 = Object(j).origin(1);
    y0 = Object(j).origin(2);
    z0 = Object(j).origin(3);
    a = Object(j).a;
    b = Object(j).b;
    c = Object(j).c;
    p = Object(j).p;
    q = Object(j).q;
    r = Object(j).r;

    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    figure(69)
    fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1)
    colormap pink
    xlim([0 200])
    ylim([-100 100])
    zlim([0 100])
end
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(['IFDS, \rho_0 = ' num2str(rho0) ', \sigma_0 = ' num2str(sigma0)],...
    'FontSize',26);
subtitle(['SF = ' num2str(sf)], 'FontSize', 24)
set(gca,'FontSize', 24)

%% ------------------------------Function---------------------------------

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
%         w = ones(numObj,1);
        for i = 1:numObj
            if i == j
                continue
            else
                w = w * (Obj(i).Gamma - 1)/...
                    ((Obj(j).Gamma - 1) + (Obj(i).Gamma - 1));
                
%                 w(i) = w(i)* prod((Field.Obj([1:i-1, i+1:end]).Gamma - 1) ./ ...
%                  ((Field.Obj(i).Gamma - 1) + (Field.Obj([1:i-1, i+1:end]).Gamma - 1)));
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

    % Calculate Weight
%     sum_w = sum(w);

%     Field.Obj.w_tilde = Field.Obj.w / sum_w;
%     Mm = sum(Field.Obj.w_tilde .* Field.Obj.M);
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

function Obj = create_scene(num, Obj, X, Y, Z)

    switch num
    case 0  % Single object
        Obj(1) = create_cone(100, 5, 0, 50, 80);
        
    case 1 % single(complex) object
        Obj(1) = create_cylinder(100, 5, 0, 25, 200);
        Obj(2) = create_pipe(60, 20, 60, 80, 5);
        Obj(3) = create_pipe(130, -30, 30, 100, 5);

    case 2 % 2 objects
        Obj(1) = create_cylinder(60, 5, 0, 30, 50);
        Obj(2) = create_sphere(120, -10, 0, 50);

    case 3 % 3 objects
        Obj(1) = create_cylinder(60, 5, 0, 30, 50);
        Obj(2) = create_sphere(120, -10, 0, 50);
        Obj(3) = create_pipe(168, 0, 0, 25, 80);

    case 12 % 12 objects
        Obj(1) = create_cylinder(100, 5, 0, 30, 50);
        Obj(2) = create_pipe(140, 20, 0, 40,10);
        Obj(3) = create_pipe(20, 20, 0, 24, 40);
        Obj(4) = create_pipe(55, -20, 0, 28, 50);
        Obj(5) = create_sphere(53, -60, 0, 50);
        Obj(6) = create_pipe(150, -80, 0, 40, 50);
        Obj(7) = create_cone(100, -35, 0, 50,45);
        Obj(8) = create_cone(170, 2, 0, 20,50);
        Obj(9) = create_cone(60, 35, 0, 50,30);
        Obj(10) = create_cylinder(110, 70, 0, 60, 50);
        Obj(11) = create_pipe(170, 60, 0, 40, 27);
        Obj(12) = create_cone(150, -30, 0, 32,45);
    case 7 % 7 objects
        Obj(1) = create_cone(60,8, 0, 70, 50);
        Obj(2) = create_cone(100,-24, 0, 89, 100);
        Obj(3) = create_cone(160,40, -4, 100, 30);
        Obj(4) = create_cone(100,100, -10, 150, 100);
        Obj(5) = create_cone(180,-70, -10, 150, 20);
        Obj(6) = create_cone(75,-75, -10, 150, 40);
        Obj(7) = create_cylinder(170, -6, 0, 34, 100);
    case 69 %pp
        Obj(1) = create_cylinder(100, 5, 0, 30, 80);
        Obj(2) = create_sphere(100, 30, 0, 40);
        Obj(3) = create_sphere(100, -20, 0, 40);
        Obj(4) = create_sphere(100, 5, 80, 30);
    end

    function Obj = create_sphere(x0, y0, z0, D)
        % Saving object's location
        Obj.origin = [x0, y0, z0];
        
        a = D/2;   b = D/2;   c = D/2;      % Object's axis length
        p = 1;     q = 1;     r = 1;        % Index parameters
       
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
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

    function Obj = create_cylinder(x0, y0, z0, D, h)
    
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 5;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
        Obj.origin = [x0, y0, z0]; 
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
    
    function Obj = create_cone(x0, y0, z0, D, h)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 0.5;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
        Obj.origin = [x0, y0, z0];
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
    
    
    function Obj = create_pipe(x0, y0, z0, D, h)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 2;     q = 2;     r = 2;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % Save to Field
        Obj.origin = [x0, y0, z0];
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

