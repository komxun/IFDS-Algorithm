% UAV Arrow
% Destination
pltDestin = scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5);

hold on, grid on, axis equal

% if animation
pltArrow = quiver3(traj{rt}(1,1), traj{rt}(2,1), traj{rt}(3,1),...
    traj{rt}(1,end)-traj{rt}(1,1), traj{rt}(2,end)-traj{rt}(2,1),...
    traj{rt}(3,end)-traj{rt}(3,1), 'ok','filled', 'LineWidth', 1.5, 'MaxHeadSize',100,'AutoScaleFactor', 2,...
    'Alignment','tail', 'MarkerSize', 12, 'MarkerFaceColor','w','ShowArrowHead','on');
% end



% IFDS Path, if available
if ~isempty(Paths{rt})
    pltPath = PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget);
end

% Trail of the UAV trajectory
if rt>1
    prevTraj = [traj{1:rt-1}];
    pltTraj = plot3(prevTraj(1,:), prevTraj(2,:), prevTraj(3,:), 'k', 'LineWidth', 1.2); 
end



% Obstacle
[Gamma, Gamma_star] = PlotObject(Object, delta_g, rt, rtsim, X, Y, Z, Gamma, Gamma_star);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); camlight

% Constraint Matrix
% imagesc(0:200, -100:100, weatherMat(:,:,rt), 'AlphaData',1)



set(gca, 'LineWidth', 2, 'FontSize', fontSize-8)
hold off
% colormap turbo
clim([0 1])

%% Functions


