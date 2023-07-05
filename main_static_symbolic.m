clc, clear, close all
tic
% Created by Komsun
% Initialization
syms X Y Z 
% assume([X Y Z], 'real')
% assumptions([X Y Z])
Field = struct('numObj', uint8(0));
% digits(2)

% Set-up Parameters
tsim = 400;          % [s] simulation total time 
dt = 0.1;            % [s] simulation time step
C = 30;              % [m/s] UAV cruising speed
targetThresh = 2.5;  % [m] allowed error for final target distance 
simMode = uint8(1);         % 1: by time, 2: by target distance
multiTarget = uint8(0);     % 1: multi-target 0: single-target]

% Starting location
Xini = 0;
Yini = 0;
Zini = 0;

% Target Destination
Xfinal = 200;
Yfinal = 0;
Zfinal = 10;

% Tuning Parameters
sf = uint8(0);   % Shape-following demand (1=on, 0=off)
rho0 = 2;        % Repulsive parameter (rho >= 0)
sigma0 = 0.01;    % Tangential parameter 

%----------- Note -------------
% Good: rho0 = 2, simga0 = 0.01
% Algorihtm stills doesnt work for overlapped objects
%------------------------------

% Saving to struct
Field.Param.sf = sf;
Field.Param.rho0 = rho0;
Field.Param.sigma0 = sigma0;
Field.C = C;

% ----- Obstacles Creation----
scene = uint8(0);
%-----------------------------
switch scene
    case 0  % Single object
        Field = create_cone(100, 5, 0, 50, 80, Field, X, Y, Z);
        
    case 1 % single(complex) object
        Field = create_cylinder(100, 5, 0, 25, 200, Field, X, Y, Z);
        Field = create_pipe(60, 20, 60, 80, 5, Field, X, Y, Z);
        Field = create_pipe(130, -30, 30, 100, 5, Field, X, Y, Z);

    case 2 % 2 objects
        Field = create_cylinder(60, 5, 0, 30, 50, Field, X, Y, Z);
        Field = create_sphere(120, -10, 0, 50, Field, X, Y, Z);

    case 3 % 3 objects
        Field = create_cylinder(60, 5, 0, 30, 50, Field, X, Y, Z);
        Field = create_sphere(120, -10, 0, 50, Field, X, Y, Z);
        Field = create_pipe(168, 0, 0, 25, 80, Field, X, Y, Z);

    case 12 % 12 objects
        Field = create_cylinder(100, 5, 0, 30, 50, Field, X, Y, Z);
        Field = create_pipe(140, 20, 0, 40,10, Field, X, Y, Z);
        Field = create_pipe(20, 20, 0, 24, 40, Field, X, Y, Z);
        Field = create_pipe(55, -20, 0, 28, 50, Field, X, Y, Z);
        Field = create_sphere(53, -60, 0, 50, Field, X, Y, Z);
        Field = create_pipe(150, -80, 0, 40, 50, Field, X, Y, Z);
        Field = create_cone(100, -35, 0, 50,45, Field, X, Y, Z);
        Field = create_cone(170, 2, 0, 20,50, Field, X, Y, Z);
        Field = create_cone(60, 35, 0, 50,30, Field, X, Y, Z);
        Field = create_cylinder(110, 70, 0, 60, 50, Field, X, Y, Z);
        Field = create_pipe(170, 60, 0, 40, 27, Field, X, Y, Z);
        Field = create_cone(150, -30, 0, 32,45, Field, X, Y, Z);
    case 7 % 7 objects
        Field = create_cone(60,8, 0, 70, 50, Field, X, Y, Z);
        Field = create_cone(100,-24, 0, 89, 100, Field, X, Y, Z);
        Field = create_cone(160,40, -4, 100, 30, Field, X, Y, Z);
        Field = create_cone(100,100, -10, 150, 100, Field, X, Y, Z);
        Field = create_cone(180,-70, -10, 150, 20, Field, X, Y, Z);
        Field = create_cone(75,-75, -10, 150, 40, Field, X, Y, Z);
        Field = create_cylinder(170, -6, 0, 34, 100, Field, X, Y, Z);
    case 69 %pp
        Field = create_cylinder(100, 5, 0, 30, 80, Field, X, Y, Z);
        Field = create_sphere(100, 30, 0, 40, Field, X, Y, Z);
        Field = create_sphere(100, -20, 0, 40, Field, X, Y, Z);
        Field = create_sphere(100, 5, 80, 30, Field, X, Y, Z);
end



colormap pink
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
xlim([0 200])
ylim([-100 100])
zlim([0 100])
title(['IFDS, \rho_0 = ' num2str(double(rho0)) ', \sigma_0 = ' num2str(double(sigma0))],...
    'FontSize',26);
set(gca,'FontSize', 24)

disp(['Number of object: ' num2str(Field.numObj)])
if sf == 0 
    disp("Shape-following: Off") 
    subtitle('Shape-following: Off', 'FontSize', 24)
elseif sf == 1
    disp("Shape-following: On")
    subtitle('Shape-following: On', 'FontSize', 24)
end

toc
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
    destin = [Xfinal Yfinal Zfinal];
end

numLine = size(destin,1);
disp("Generating paths for " + num2str(numLine) + " destinations . . .")


for L = 1:numLine
    
    disp("Calculating path for destination #" + num2str(L))
    xd = (destin(L,1));
    yd = (destin(L,2));
    zd = (destin(L,3));

    [UBar, n, u] = calc_ubar(X, Y, Z, xd, yd, zd, Field);    

    %% Path for streamline
   
    Wp = zeros(3,tsim+1);
    Wp(:,1) = [Xini; Yini; Zini];

    switch simMode
        case 1 % Simulate by time
            disp("Simulating by time for " + num2str(tsim) + " s.")
            for j = 1:tsim
            
                xx = Wp(1,j);
                yy = Wp(2,j);
                zz = Wp(3,j);

                ntu =  double(n(xx,yy,zz)'*u(xx,yy,zz));
       
                if norm([xx yy zz] - double([xd yd zd])) < targetThresh
                    disp('Target destination reached!')
                    Wp = Wp(:,1:j);
                    break
                else
                    if ntu < 0 || sf == 1
            %             disp('case 1 activated')
                        Wp(:,j+1) = Wp(:,j) + double(UBar(Wp(1,j), Wp(2,j), Wp(3,j))) * dt;
            
                    elseif ntu >= 0 && sf == 0
            %             disp('case 2 activated')
                        Wp(:,j+1) = Wp(:,j) + double(u(Wp(1,j), Wp(2,j), Wp(3,j))) * dt;
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


%% ------------------------------Function---------------------------------

function [UBar, n, u]  = calc_ubar(X, Y, Z, xd, yd, zd, Field)

    rho0   = Field.Param.rho0;
    sigma0 = Field.Param.sigma0;
    C = Field.C;

    dist = sqrt((X - xd)^2 + (Y - yd)^2 + (Z - zd)^2);

    u(X,Y,Z) = vpa(-[C*(X - xd)/dist, C*(Y - yd)/dist, C*(Z - zd)/dist]');
    
    %% Components
    numObj = Field.numObj;

    Mm = zeros(3);
    sum_w = 0;

    for j = 1:numObj
        dGdx = Field.Obj(j).dGdx;
        dGdy = Field.Obj(j).dGdy;
        dGdz = Field.Obj(j).dGdz;
        % Reading Gamma for each object
        Gamma = Field.Obj(j).Gamma;
        
        % Unit normal vector
        n = [dGdx; dGdy; dGdz]; 
    
        % Unit tangential vector
        t = [dGdy; -dGdx; 0];
    
        % Object Distance from UAV
        x0 = Field.Obj(j).origin(1);
        y0 = Field.Obj(j).origin(2);
        z0 = Field.Obj(j).origin(3);
    
        % Calculate parameters
        [rho, sigma, dist_obj] = calc_params();
    
        % Modular Matrix (Perturbation Matrix
        M = vpa(eye(3) - n*n'/(abs(Gamma)^(1/rho)*(n')*n)...
        + t*n'/(abs(Gamma)^(1/sigma)*norm(t)*norm(n)));  % tao is removed for now
      
        % Weight
        w = 1;
%         w = ones(numObj,1);
        for i = 1:numObj
            if i == j
                continue
            else
                w = w * (Field.Obj(i).Gamma - 1)/...
                    ((Field.Obj(j).Gamma - 1) + (Field.Obj(i).Gamma - 1));
                
%                 w(i) = w(i)* prod((Field.Obj([1:i-1, i+1:end]).Gamma - 1) ./ ...
%                  ((Field.Obj(i).Gamma - 1) + (Field.Obj([1:i-1, i+1:end]).Gamma - 1)));
            end
        end
    
        sum_w = sum_w + w;

        % Saving to Field
        Field.Obj(j).n = n;
        Field.Obj(j).t = t;
        Field.Obj(j).dist = dist_obj;
        Field.Obj(j).rho = rho;
        Field.Obj(j).sigma = sigma;
        Field.Obj(j).M  = M;
        Field.Obj(j).w = w;
    
    end

    % Calculate Weight
%     sum_w = sum(w);

%     Field.Obj.w_tilde = Field.Obj.w / sum_w;
%     Mm = sum(Field.Obj.w_tilde .* Field.Obj.M);
    for j = 1:numObj
        Field.Obj(j).w_tilde = Field.Obj(j).w/sum_w;
        Mm = Mm + vpa(Field.Obj(j).w_tilde * Field.Obj(j).M);
    end

    UBar = (Mm*u) 

    function [rho, sigma, dist_obj] = calc_params()
        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);
        
        % Reactivity Parameter 
        rho = rho0 * exp(1 - 1/(dist_obj * dist));
        sigma = sigma0 * exp(1 - 1/(dist_obj * dist));
    end

end



%% ------------------------Object Modeling functions----------------------
function Field = create_sphere(x0, y0, z0, D, Field, X, Y, Z)
    
    Field.numObj = Field.numObj + 1;
    
    % Saving object's location
    Field.Obj(Field.numObj).origin = [x0, y0, z0]; 

    % Object's origin point
    Field.Obj(Field.numObj).origin = [x0, y0, z0]; 
    
    % Object's axis length
    a = D/2;   b = D/2;   c = D/2;
    
    % Index parameters
    p = 1;   q = 1;   r = 1;
   
    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    
    % Differential
    [dGdx, dGdy, dGdz] = calc_dG();

    % Save to Field
    Field.Obj(Field.numObj).Gamma = Gamma;
    Field.Obj(Field.numObj).dGdx = vpa(dGdx);
    Field.Obj(Field.numObj).dGdy = vpa(dGdy);
    Field.Obj(Field.numObj).dGdz = vpa(dGdz);
    
    % Plot the surface
    figure(69);
    fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1)
    hold on, axis equal;
    function [dGdx, dGdy, dGdz] = calc_dG()
        dGdx(X,Y,Z) = (2*p*((X - x0)/a).^(2*p - 1))/a;
        dGdy(X,Y,Z) = (2*q*((Y - y0)/b).^(2*q - 1))/b;
        dGdz(X,Y,Z) = (2*r*((Z - z0)/c).^(2*r - 1))/c;

%         dGdx = diff(Gamma,X);
%         dGdy = diff(Gamma,Y);
%         dGdz = diff(Gamma,Z);
    end

end

function Field = create_cylinder(x0, y0, z0, D, h, Field, X, Y, Z)
    
    Field.numObj = Field.numObj + 1;
    
    % Saving object's location
    Field.Obj(Field.numObj).origin = [x0, y0, z0]; 
    
    % Object's axis length
    a = D/2;   b = D/2;   c = h;
    
    % Index parameters
    p = 1;   q = 1;   r = 5;
    
    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    
    % Differential
    [dGdx, dGdy, dGdz] = calc_dG();

    % Save to Field
    Field.Obj(Field.numObj).Gamma = Gamma;
    Field.Obj(Field.numObj).dGdx = vpa(dGdx);
    Field.Obj(Field.numObj).dGdy = vpa(dGdy);
    Field.Obj(Field.numObj).dGdz = vpa(dGdz);
    

    % Plot the surface
    figure(69);
    fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1)
    hold on, axis equal;

    function [dGdx, dGdy, dGdz] = calc_dG()
        dGdx(X,Y,Z) = (2*p*((X - x0)/a).^(2*p - 1))/a;
        dGdy(X,Y,Z) = (2*q*((Y - y0)/b).^(2*q - 1))/b;
        dGdz(X,Y,Z) = (2*r*((Z - z0)/c).^(2*r - 1))/c;

%         dGdx = diff(Gamma,X);
%         dGdy = diff(Gamma,Y);
%         dGdz = diff(Gamma,Z);
    end
end

function Field = create_cone(x0, y0, z0, D, h, Field, X, Y, Z)

    Field.numObj = Field.numObj + 1;

    % Saving object's location
    Field.Obj(Field.numObj).origin = [x0, y0, z0]; 
    
    % Object's axis length
    a = D/2;   b = D/2;   c = h;
    
    % Index parameters
    p = 1;   q = 1;   r = 0.5;
 
    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    
    % Differential
    [dGdx, dGdy, dGdz] = calc_dG();
    
    % Save to Field
    Field.Obj(Field.numObj).Gamma = Gamma;
    Field.Obj(Field.numObj).dGdx = vpa(dGdx);
    Field.Obj(Field.numObj).dGdy = vpa(dGdy);
    Field.Obj(Field.numObj).dGdz = vpa(dGdz);

    % Plot the surface
    figure(69);
    fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1)
    hold on, axis equal;

    function [dGdx, dGdy, dGdz] = calc_dG()
        dGdx(X,Y,Z) = (2*p*((X - x0)/a).^(2*p - 1))/a;
        dGdy(X,Y,Z) = (2*q*((Y - y0)/b).^(2*q - 1))/b;
        dGdz(X,Y,Z) = (2*r*((Z - z0)/c).^(2*r - 1))/c;

%         dGdx = diff(Gamma,X);
%         dGdy = diff(Gamma,Y);
%         dGdz = diff(Gamma,Z);
    end

end


function Field = create_pipe(x0, y0, z0, base, h, Field, X, Y, Z)

    Field.numObj = Field.numObj + 1;
    
    % Saving object's location
    Field.Obj(Field.numObj).origin = [x0, y0, z0]; 

    % Object's axis length
    a = base/2;   b = base/2;   c = h;
    
    % Index parameters
    p = sym(2);   q = sym(2);   r = sym(2);

    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);

    % Differential
    [dGdx, dGdy, dGdz] = calc_dG();
    
    % Save to Field
    Field.Obj(Field.numObj).Gamma = Gamma;
    Field.Obj(Field.numObj).dGdx = vpa(dGdx);
    Field.Obj(Field.numObj).dGdy = vpa(dGdy);
    Field.Obj(Field.numObj).dGdz = vpa(dGdz);
    
    % Plot the surface
    figure(69);
    fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1)
    hold on, axis equal;

    function [dGdx, dGdy, dGdz] = calc_dG()
        dGdx(X,Y,Z) = (2*p*((X - x0)/a).^(2*p - 1))/a;
        dGdy(X,Y,Z) = (2*q*((Y - y0)/b).^(2*q - 1))/b;
        dGdz(X,Y,Z) = (2*r*((Z - z0)/c).^(2*r - 1))/c;

%         dGdx = diff(Gamma,X);
%         dGdy = diff(Gamma,Y);
%         dGdz = diff(Gamma,Z);
    end
end

