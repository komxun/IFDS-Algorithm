clc, clear, close all

load("kuy.mat")

% Set-up Parameters
fontSize = 26;
showDisp = 1;
tsim = uint16(400);          % [s] simulation time for the path 
rtsim = 1;                   % [s] (50) time for the whole scenario 
dt = 0.1;            % [s] simulation time step
C  = 30;             % [m/s] UAV cruising speed
targetThresh = 2.5;  % [m] allowed error for final target distance 
simMode = uint8(1);          % 1: by time, 2: by target distance
multiTarget = uint8(0);      % 1: multi-target 0: single-target
scene = 1;       % Scenario selection
                % 0) NO object 1) 1 object, 2) 2 objects 
                % 3) 3 objects 4) 3 complex objects
                % 7) non-urban 12) urban environment
% Path's Starting location
Xini = 0;
Yini = 0;
Zini = 10;

% Target Destination
Xfinal = 200;
Yfinal = 0;
Zfinal = 20;
destin = [Xfinal Yfinal Zfinal];
C  = 30;             % [m/s] UAV cruising speed



%%


kappa = 100;
delta = 50;
kd = 0;
tuning = [kappa, delta, kd]; 

leg = ["Desired Path"];

figure(70)
PlotPath(1, Paths, Xini, Yini, Zini, destin, multiTarget), hold on, grid minor
num = 6;
traj = cell(num,rtsim);
for i = 1:num
    % UAV's Initial State
    x_i = 0;
    y_i = -50;
    z_i = 0;
    psi_i = 0;          % [rad] Initial Yaw angle
    gamma_i = 0;        % [rad] Initial Pitch angle
    trajectory = [x_i; y_i; z_i];

    for j = 1:length(Paths{1,:})-1
    
        Wfm1 = Paths{1}(:,j);
    %     Wi = [x_i; y_i; z_i];
        Wi = Wfm1;
        Wf = Paths{1}(:,j+1);
    
        path_vect = Wf - Wfm1;
        a = path_vect(1);
        b = path_vect(2);
        c = path_vect(3);
        
        % Check if the waypoint is ahead of current position
        if a*(x_i - Wf(1)) + b*(y_i - Wf(2)) + c*(z_i - Wf(3)) < 0
    
            [x, y, z, psi, gamma] = CCA3D_straight(Wi, Wf, x_i, y_i, z_i, psi_i, gamma_i, C, Wfm1, tuning);
            x_i = x(end);
            y_i = y(end);
            z_i = z(end);
            psi_i = psi(end);
            gamma_i = gamma(end);
        
            trajectory = [trajectory, [x y z]']; 
        else
%             disp("skip waypoint #" + num2str(j)) 
        end
        
    
    end
    trajectory;
    plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'LineWidth', 2)
    leg = [leg; "\kappa = " + num2str(tuning(1))];
    legend(leg)

    tuning(1) = tuning(1) + 1;

end




xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
set(gca, 'FontSize', fontSize, 'LineWidth', 2)
title("3D Carrot-Chasing Algorithm, \delta = " + num2str(delta))

%%

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
        plot3(Paths{1,rt}(1,:), Paths{1,rt}(2,:), Paths{1,rt}(3,:),'b--', 'LineWidth', 3)
        hold on, grid on, grid minor, axis equal
    end

    xlim([0 200])
    ylim([-60 20])
%     zlim([0 50])
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
%     hold off
end