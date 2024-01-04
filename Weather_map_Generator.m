clc, clear, close all

saveVid = 0;   % 1: save  2: no

fontSize = 24;

% number of point
Nx = 200;
Ny = 200;
tmax = 30;

weatherMat = zeros(Nx, Ny, tmax);
% numSeed = 8187;
numSeed = 69;

figure(88)
axis equal tight
colormap turbo

for t = 1:tmax
    f = 0.015;   % Increase the randomness control parameter
%     f = 0.05;     % Lower = higher frequency
    rand('seed', numSeed)
    
    nx1 = 1 + floor(0.999*f*Nx);
    nx2 = Nx - nx1 + 1;
    ny1 = 1 + floor(0.999*f*Nx);
    ny2 = Nx - nx1 + 1;
    
    F = rand(Nx,Ny);
    F = fft2(F);   % 2D FFT
    F(nx1:nx2 - 0.5*sin(t/2), :) = 0;
    F(:, ny1:ny2 - 0.5*cos(t/2)) = 0;
    
    F = real(ifft2(F));  % 2D iFFT
    
    % Scale the values to the desired range (0.2 - 1)
    F = (F - min(F(:))) ./ (max(F(:)) - min(F(:)));
    F = F * 1.22;
%     F = F * 1.15;
    F(F>1) = 1;

    imagesc(F)
    
    hold on 
%     [C2,h2] = contourf(1:200, 1:200, F, [1, 1], 'FaceAlpha',0,...
%         'LineColor', 'w', 'LineWidth', 2);
    
%     clabel(C2,h2,'FontSize',15,'Color','w')
    axis equal tight
    
    
    title("numSeed = " + num2str(numSeed) + ", f = " + num2str(f)...
        + num2str(t,', time = %4.1f s'))
    set(gca, 'YDir', 'normal', 'FontSize', fontSize)
    colorbar
    pause(0.01)
    hold off

    % Video saving
    if saveVid
        frm(t) = getframe(gcf) ;
        drawnow
    end

    weatherMat(:,:,t) = F;
end

set(gca, 'YDir', 'normal', 'LineWidth', 2)
fileName = "WeatherMat_" + num2str(numSeed) +".mat";
% save(fileName, 'weatherMat')

if saveVid
    video_name = "WeatherMat_" + num2str(numSeed) + ".avi";
    % create the video writer with 1 fps
    writerObj = VideoWriter(video_name);
    writerObj.FrameRate = 30;
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


