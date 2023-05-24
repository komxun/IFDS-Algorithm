clc, clear, close all

global numObj Field
numObj = 0;
Field = struct;
% Set-up Parameters
tsim = 200; % seconds
sf = 0; % Shape-following demand (1=on, 0=off)

% Starting location
Xini = 0;
Yini = 0;
Zini = 5;

% Target Destination
xd = sym(-200);
yd = sym(0);
zd = sym(-30);

% Tuning Parameters
rho0 = sym(2);        % Repulsive parameter (rho >= 0)
sigma0 = sym(0.03);    % Tangential parameter 

% Obstacles Creation
Field.Obj(1).Gamma = create_cylinder(100, 5, 0, 30, 50);
Field.Obj(2).Gamma = create_pipe(140, 20, 0, 40,10);
Field.Obj(3).Gamma = create_pipe(20, 20, 0, 24, 40);
Field.Obj(4).Gamma = create_pipe(55, -20, 0, 28, 50);
Field.Obj(5).Gamma = create_sphere(53, -60, 0, 50);
Field.Obj(6).Gamma = create_pipe(150, -80, 0, 40, 50);
Field.Obj(7).Gamma = create_cone(100, -35, 0, 50,45);
Field.Obj(8).Gamma = create_cone(170, 2, 0, 20,50);
Field.Obj(9).Gamma = create_cone(60, 35, 0, 50,30);
Field.Obj(10).Gamma = create_cylinder(110, 70, 0, 60, 50);
Field.Obj(11).Gamma = create_pipe(170, 60, 0, 40, 27);
Field.Obj(12).Gamma = create_cone(150, -30, 0, 32,45);

xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([0 200])
ylim([-100 100])
zlim([0 50])
title(['IFDS, \rho_0 = ' num2str(double(rho0)) ', \sigma_0 = ' num2str(double(sigma0))],...
    'FontSize',24);


% UAV position XYZ
syms X Y Z
assume([X Y Z], 'real')
% assumptions([X Y Z])

disp(['Number of object: ' num2str(numObj)])
if sf == 0 
    disp("Shape-following: Off") 
    subtitle('Shape-following: Off', 'FontSize', 22)
elseif sf == 1
    disp("Shape-following: On")
    subtitle('Shape-following: On', 'FontSize', 22)
end

%% Original Fluid
C = sym(2);   % [m/s]?UAV cruising speed

dist(X,Y,Z) = sqrt((X - xd)^2 + (Y - yd)^2 + (Z - zd)^2);

u(X,Y,Z) = [C*(X - xd)/dist; C*(Y - yd)/dist; C*(Z - zd)/dist] ;

%% Components

UBar = 0;
Mm = zeros(3);

for j = 1:numObj
    
    % Reading Gamma for each object
    Gamma = Field.Obj(j).Gamma;

    dGdx = diff(Gamma, X);
    dGdy = diff(Gamma, Y);
    dGdz = diff(Gamma, Z);

    % Unit normal vector
    n = [dGdx; dGdy; dGdz]; 

    % Unit tangential vector
    t = [dGdy; -dGdx; 0];

    % Object Distance from UAV
    x0 = Field.Obj(j).origin(1);
    y0 = Field.Obj(j).origin(2);
    z0 = Field.Obj(j).origin(3);

    dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);

    % Reactivity Parameter 
    rho = rho0 * exp(1 - 1/(dist_obj * dist));
    sigma = sigma0 * exp(1 - 1/(dist_obj * dist));

    % Modular Matrix (Perturbation Matrix
    M = eye(3) - n*n'/(abs(Gamma)^(1/rho)*(n')*n)...
    + t*n'/(abs(Gamma)^(1/sigma)*norm(t)*norm(n));  % tao is removed for now

    % Interfered Fluid Velocity
    ubar = calc_ubar(tsim, sf, n, u, M);


    % Weight
    w = sym(1);
    for i = 1:numObj
        if i == j
            continue
        else
            w = w * (Field.Obj(i).Gamma - 1)/...
                ((Field.Obj(j).Gamma - 1) + (Field.Obj(i).Gamma - 1));
        end
    end

    % Saving to Field
    Field.Obj(j).n = n;
    Field.Obj(j).t = t;
    Field.Obj(j).dist = dist_obj;
    Field.Obj(j).rho = rho;
    Field.Obj(j).sigma = sigma;
    Field.Obj(j).M  = M;
    Field.Obj(j).ubar = ubar;
    Field.Obj(j).w = w;

    % Calculating weighted final Fluid Velocity
%     UBar = UBar + w*ubar;
    Mm = Mm + w*M;
end

UBar = Mm*u;

%% Calculate Weight


%% Path for streamline

dt = 1;
Wp = zeros(3,tsim+1);
Wp(:,1) = [Xini; Yini; Zini];

for j = 1:tsim

    xx = Wp(1,j);
    yy = Wp(2,j);
    zz = Wp(3,j);

    Wp(:,j+1) = Wp(:,j) + double(UBar(Wp(1,j), Wp(2,j), Wp(3,j))) * dt;
    
end

%% Plotting the path
figure(69)

plot3(Wp(1,:), Wp(2,:), Wp(3,:), 'LineWidth', 1.5)
% hold on, grid on

scatter3(Wp(1,1), Wp(2,1), Wp(3,1), 'filled', 'r')
xlabel('X'); ylabel('Y'); zlabel('Z');

% scatter3(-xd, -yd, -zd, 'xr')
hold off

figure
subplot(3,1,1)
plot(Wp(1,:),'o-')
subplot(3,1,2)
plot(Wp(2,:),'o-')
subplot(3,1,3)
plot(Wp(3,:),'o-')

%% Function

function Gamma = create_sphere(x0, y0, z0, D)
    global numObj Field
    numObj = numObj + 1;
    % create_sphere(x0,y0,z0,D)
    
    % Saving object's location
    Field.Obj(numObj).origin = [x0, y0, z0]; 
    % Object's origin point
    x0 = sym(x0);   y0 = sym(y0);   z0 = sym(z0);
    
    % Object's axis length
    a = sym(D/2);   b = sym(D/2);   c = sym(D/2);
    
    % Index parameters
    p = sym(1);   q = sym(1);   r = sym(1);

    syms X Y Z
    assume([X Y Z], 'real')
    % assumptions([X Y Z])
    
    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    Object = Gamma == 1;

    % Plot the surface
    figure(69);

    fimplicit3(Object)
    % colorbar
    colormap pink
    hold on, axis equal;
end

function Gamma = create_cylinder(x0, y0, z0, D, h)
    global numObj Field
    numObj = numObj + 1;
    % create_sphere(x0,y0,z0,D)
    
    % Saving object's location
    Field.Obj(numObj).origin = [x0, y0, z0]; 
    % Object's origin point
    x0 = sym(x0);   y0 = sym(y0);   z0 = sym(z0);
    
    % Object's axis length
    a = sym(D/2);   b = sym(D/2);   c = sym(h);
    
    % Index parameters
    p = sym(1);   q = sym(1);   r = sym(5);

    syms X Y Z
    assume([X Y Z], 'real')
    % assumptions([X Y Z])
    
    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    Object = Gamma == 1;

    % Plot the surface
    figure(69);

    fimplicit3(Object)
    % colorbar
    colormap pink
    hold on, axis equal;
end

function Gamma = create_cone(x0, y0, z0, D, h)
    global numObj Field
    numObj = numObj + 1;
    % create_sphere(x0,y0,z0,D)
    
    % Saving object's location
    Field.Obj(numObj).origin = [x0, y0, z0]; 
    % Object's origin point
    x0 = sym(x0);   y0 = sym(y0);   z0 = sym(z0);
    
    % Object's axis length
    a = sym(D/2);   b = sym(D/2);   c = sym(h);
    
    % Index parameters
    p = sym(1);   q = sym(1);   r = sym(0.5);

    syms X Y Z
    assume([X Y Z], 'real')
    % assumptions([X Y Z])
    
    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    Object = Gamma == 1;

    % Plot the surface
    figure(69);

    fimplicit3(Object)
    % colorbar
    colormap pink
    hold on, axis equal;
end

function Gamma = create_pipe(x0, y0, z0, base, h)
    global numObj Field
    numObj = numObj + 1;
    % create_sphere(x0,y0,z0,D)
    
    % Saving object's location
    Field.Obj(numObj).origin = [x0, y0, z0]; 
    % Object's origin point
    x0 = sym(x0);   y0 = sym(y0);   z0 = sym(z0);
    
    % Object's axis length
    a = sym(base/2);   b = sym(base/2);   c = sym(h);
    
    % Index parameters
    p = sym(4);   q = sym(4);   r = sym(4);

    syms X Y Z
    assume([X Y Z], 'real')
    % assumptions([X Y Z])
    
    % Object Shape Equation
    Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
    Object = Gamma == 1;

    % Plot the surface
    figure(69);

    fimplicit3(Object)
    % colorbar
    colormap pink
    hold on, axis equal;
end

function ubar = calc_ubar(tsim, sf, n, u, M)
    Wp = zeros(3,tsim+1);
    for j = 1:tsim
    
        x = Wp(1,j);
        y = Wp(2,j);
        z = Wp(3,j);
    
        if n(x,y,z)'*u(x,y,z) < 0 || sf == 1
    %         disp('case 1 activated')
            Mm = M(x,y,z);
        elseif n(x,y,z)'*u(x,y,z) >= 0 && sf == 0
    %         disp('case 2 activated')
            Mm = eye(3);
        end
        
        ubar = Mm*u;
%         Wp(:,j+1) = Wp(:,j) + double(ubar(Wp(1,j), Wp(2,j), Wp(3,j))) * dt;
    end   
end