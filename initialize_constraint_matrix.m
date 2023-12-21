% load WeatherMat_3.mat   % Good! (ok for rt = 100!)
% load WeatherMat_8187.mat   % Good for static plot (issue in rt>20)
% load WeatherMat_321.mat
s = load("elev.mat");
elev = double(s.A);
elev = elev(1:mapSpan, 1:mapSpan);

% weatherMat = (elev - min(elev(:))) ./ (max(elev(:)) - min(elev(:)));

weatherMat = elev;


warning off
weatherMatMod = weatherMat;
weatherMatMod(weatherMatMod<Zini) = 0;
% weatherMatMod(weatherMatMod<B_L) = B_L;
% weatherMatMod(weatherMatMod>B_U) = 1;


xspace = 1:mapSpan;
yspace = 1:mapSpan;
[xgrid, ygrid] = meshgrid(xspace, yspace);

WMCell = cell(1,size(weatherMat,3));
dwdxCell = cell(1,size(weatherMat,3));
dwdyCell = cell(1,size(weatherMat,3));

for j = 1:size(weatherMat,3)
    % WMCell{j} = griddedInterpolant(weatherMat(:,:,j)');
    WMCell{j} = griddedInterpolant(weatherMatMod(:,:,j)');
    z_values = WMCell{j}(xgrid,ygrid);
    [grad_x, grad_y] = gradient(z_values, xspace, yspace);
    dwdxCell{j} = griddedInterpolant(grad_x');
    dwdyCell{j} = griddedInterpolant(grad_y');
end


figure(88)
subplot(1,2,1)
set(gca, 'YDir', 'normal')
colormap turbo
contourf(1:mapSpan,1:mapSpan,weatherMat(:,:,1), 30)
axis equal tight
colorbar
hold on 
[C2,h2] = contourf(1:mapSpan, 1:mapSpan, weatherMat(:,:,1), [1, 1], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
clabel(C2,h2,'FontSize',15,'Color','w')
title("Original Constraint Matrix")
set(gca, 'YDir', 'normal', 'FontSize', fontSize)

subplot(1,2,2)
set(gca, 'YDir', 'normal')
colormap turbo
contourf(1:mapSpan,1:mapSpan,weatherMatMod(:,:,1), 30)
hold on 
[C2,h2] = contourf(1:mapSpan, 1:mapSpan, weatherMat(:,:,1), [B_U, B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);



title("Filtered Constraint Matrix: B_L = " + num2str(B_L) + ", B_U = " + num2str(B_U))
set(gca, 'YDir', 'normal', 'FontSize', fontSize)
axis equal tight
colorbar


% Gradient
figure
% Define the X, Y, and Z coordinates
X = 1:mapSpan;
Y = 1:mapSpan;
Z = zeros(length(Y), length(X));

% Compute the gradient of the matrix numerically
dwdx = double(diff(weatherMat, 1, 2));
dwdy = double(diff(weatherMat, 1, 1));
% dwdz = diff(weatherMat, 1, 1);

% Pad the gradient matrices to match the size of the original matrix
dwdx = [dwdx, zeros(size(dwdx, 1), 1)];
dwdy = [dwdy; zeros(1, size(dwdy, 2))];


% Plot the gradient vectors
[X_grid, Y_grid] = meshgrid(X, Y);
quiver(X_grid(1:5:end), Y_grid(1:5:end), dwdx(1:5:end), dwdy(1:5:end),2);
axis equal tight;

% Set the correct axis limits and labels
xlim([0, mapSpan]);
% ylim([-100, 100]);
xlabel('X');
ylabel('Y');


% Add a title
title('Numerical Gradient of the Matrix');