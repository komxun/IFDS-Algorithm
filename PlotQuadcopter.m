function h = PlotQuadcopter(pos, heading, armLength, bodyColor)
% PlotQuadcopter  Draws a small quadcopter (X-frame) marker at pos.
%
%   h = PlotQuadcopter(pos, heading, armLength, bodyColor)
%
%   pos       : 1x3 or 3x1 world position [x y z]
%   heading   : 1x3 or 3x1 heading vector (yaw is taken from XY projection).
%               If omitted or zero, yaw = 0 (body x aligned with world +X).
%   armLength : half-diagonal of the X-frame in metres (default = 4).
%   bodyColor : colour of the airframe arms        (default = 'k').
%
%   Returns array of graphics handles so the caller can delete/update them.
%
%   The front rotors are drawn red, the rear rotors match bodyColor, and a
%   small green stub indicates the body +X (forward) direction.

    if nargin < 2 || isempty(heading),   heading   = [1 0 0]; end
    if nargin < 3 || isempty(armLength), armLength = 4;       end
    if nargin < 4 || isempty(bodyColor), bodyColor = 'w';     end

    pos = pos(:);                       % 3x1

    % --- Yaw from heading projected onto XY plane ----------------------
    psi = atan2(heading(2), heading(1));
    if ~isfinite(psi), psi = 0; end
    Rz  = [cos(psi) -sin(psi) 0;
           sin(psi)  cos(psi) 0;
           0         0        1];

    % --- Motor positions in body frame (X configuration) ---------------
    L      = armLength;
    angles = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];   % FR, FL, RL, RR
    motorsB = [L*cos(angles); L*sin(angles); zeros(1,4)];
    motorsW = Rz*motorsB + pos;                % world frame

    h = gobjects(0);

    % --- Arms: motor1<->motor3, motor2<->motor4 ------------------------
    h(end+1) = plot3(motorsW(1,[1 3]), motorsW(2,[1 3]), motorsW(3,[1 3]), ...
                     '-', 'Color', bodyColor, 'LineWidth', 2);
    h(end+1) = plot3(motorsW(1,[2 4]), motorsW(2,[2 4]), motorsW(3,[2 4]), ...
                     '-', 'Color', bodyColor, 'LineWidth', 2);

    % --- Rotor disks ---------------------------------------------------
    nseg   = 24;
    th     = linspace(0, 2*pi, nseg);
    rotorR = 0.45*L;
    cTh    = rotorR*cos(th);
    sTh    = rotorR*sin(th);
    for i = 1:4
        cx = motorsW(1,i); cy = motorsW(2,i); cz = motorsW(3,i);
        xs = cx + cTh;
        ys = cy + sTh;
        zs = cz*ones(size(th));
        if i == 1 || i == 4   % front rotors
            % faceCol = [1.0 0.2 0.2];  % red
            % faceCol = [0.9 0 0];  % red
            faceCol = "c";
            % faceCol = [1.0 1.0 1.0];  % white
        else
            faceCol = bodyColor;
        end
        h(end+1) = fill3(xs, ys, zs, faceCol, ...
                         'FaceAlpha', 0.55, ...
                         'EdgeColor', 'k', ...
                         'LineWidth', 1.2); %#ok<AGROW>
    end

    % --- Heading stub (body +X) ---------------------------------------
    fwd = Rz*[1.3*L; 0; 0] + pos;
    h(end+1) = plot3([pos(1) fwd(1)], [pos(2) fwd(2)], [pos(3) fwd(3)], ...
                     '-', 'Color', [0.1 0.7 0.2], 'LineWidth', 1.8);
end
