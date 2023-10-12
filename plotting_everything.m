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
colormap turbo
clim([0 1])

%% Functions
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

%         if rtsim > 1
%             fimplicit3(Gamma == 1,'EdgeColor','k','FaceAlpha',1,'MeshDensity',20), hold on
%             fimplicit3(Gamma_star == 1, 'EdgeColor','k','FaceAlpha',0,'MeshDensity',20)
%         else
            fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1,'MeshDensity',80), hold on
            fimplicit3(Gamma_star == 1, 'EdgeColor','none','FaceAlpha',0.2,'MeshDensity',30)
%         end

        xlim([0 200])
        ylim([-100 100])
        zlim([0 100])
    end

end

function pltPath = PlotPath(rt, Paths, Xini, Yini, Zini, destin, multiTarget)
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
        pltPath = plot3(Paths{1,rt}(1,:), Paths{1,rt}(2,:), Paths{1,rt}(3,:),'b--', 'LineWidth', 1.8);
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