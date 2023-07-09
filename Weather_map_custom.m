clc, clear, close all

% Function Size
Sx = 2; % [m]
Sy = 2; % [m]

% number of point
Nx = 200;
Ny = round(Nx*Sy/Sx);

% Generate Random Function


figure(88)
axis equal tight
colormap turbo

for t = 1:50
    f = 0.02;   % control the randomness  
            % higher = faster randomness (higher noise frequency)
    rand('seed',69)
    % 0.999 to make sure f is less than 1
    nx1 = 1 + floor(0.999*f*Nx);
    nx2 = Nx - nx1 + 1;
    ny1 = 1 + floor(0.999*f*Nx);
    ny2 = Nx - nx1 + 1;
    
    F = rand(Nx,Ny);
    F = fft2(F);   % 2D FFT
    F(nx1:nx2 - 0.5*sin(t/2), :) = 0;
    F(:, ny1:ny2 - 0.5*cos(t/2)) = 0;
    
    F = real(ifft2(F));  % 2D iFFT
    
    % Function Axes
    dx = Sx/Nx;
    xa = [0.5:Nx-0.5]*dx;
    dy = Sy/Ny;
    ya = [0.5:Ny-0.5]*dy;
    
    % Draw Function
    % imagesc(xa,ya,F.')
    
%     colorbar
    imagesc(F)
    title(num2str(t,'time = %4.1f s'))
    set(gca, 'YDir', 'normal')
    colorbar
    pause(0.1)
    
    
end


set(gca, 'YDir', 'normal')

