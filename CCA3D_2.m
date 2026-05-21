function [x_final, y_final, z_final, vm, psi_final, gamma_final, timeSpent] = CCA3D_2(Wi, Wf, x0, y0, z0, psi0, gamma0, v0, V_ref, tuning)
% CCA3D_2  Carrot-Chasing path-following with translational point-mass
%          dynamics (thrust, drag, lift, gravity).
%
% Inputs :  Wi, Wf      initial / final waypoint                [3x1, m]
%           x0,y0,z0    initial UAV position                    [m]
%           psi0,gamma0 initial yaw / pitch                     [rad]
%           v0          initial UAV speed (handoff from prev.   [m/s]
%                       segment, preserves continuity)
%           V_ref       cruise reference speed (target)         [m/s]
%           tuning      [kappa, delta, kd] CCA gains
%
% Outputs:  x_final,y_final,z_final  final position             [m]
%           vm           velocity time-history v(t)             [1xN, m/s]
%           psi_final,gamma_final    final attitude angles      [rad]
%           timeSpent    elapsed sim time                       [s]

dt = 0.01;
timeSpent = 0;
animation = 0;
%.. Time
t(1) = 0 ;                 % Simulation Time [s]

%% .. UAV Physical Parameters (matching SE(3) controller, Crazyflie 2.X)
m     = 1.5;               % [kg]      mass (3DR Iris)
g_acc = 9.81;              % [m/s^2]   gravity
% Aerodynamic parameters (small quadrotor reference area / coefficients)
rho   = 1.225;             % [kg/m^3]  air density (sea level)
Cd    = 1.0;               % [-]       drag coefficient (bluff body)
Cl    = 0.5;               % [-]       lift coefficient (informational)
S     = 0.01;              % [m^2]     reference frontal area
T_max = 3 * m * g_acc;     % [N]       maximum thrust  (T/W = 3)
Kv    = 2.0;               % [1/s]     speed-tracking gain (P controller)

%% .. Initial Velocity (continuous handoff from previous segment)
v(1)   = max(v0, 0.1);     % avoid v = 0 (singularity in dpsi = u1/(v cos g))
vm(1)  = v(1);             % velocity history

%.. Maximum Lateral Acceleration of UAV
Rmin = 10;                 % UAV Minimum Turn Radius [m]  (lower is better)
% Rmin = 13 ;              % UAV Minimum Turn Radius [m]
umax = v(1)^2 / Rmin ;     % UAV Maximum Lateral Acceleration [m/s^2]
del_psi(1) = 0;

%-----------------------Design Parameters---------------------------------
kappa = tuning(1);
delta = tuning(2);
kd = tuning(3);
% ------------------------------------------------------------------------

% Path Following Algorithm - CCA (Carrot Chasing Algorithm)

i = 0 ;                 % Time Index

x(1) = x0;                    % Initial UAV X Position [m]
y(1) = y0;                    % Initial UAV Y Position [m]
z(1) = z0;
psi(1) = psi0 ;               % Initial UAV Heading Angle [rad]
gamma(1) = gamma0;            % Initial UAV Pitch Angle [rad]
p(:,1) = [ x(1), y(1), z(1) ]' ;    % UAV Position Initialization [m]


% -------------------------------------------
% Normal plane checker
% Find an equation of the plane through the point Wf and perpendicular to
% the vector (Wf - Wi)
Rw_vect = Wf - Wi;
ox = Wf(1);
oy = Wf(2);
oz = Wf(3);
a = Rw_vect(1);
b = Rw_vect(2);
c = Rw_vect(3);

check = 1;

    while a*(x(i+1) - ox) + b*(y(i+1) - oy) + c*(z(i+1) - oz) < 0
%     while (x(i+1) < Wf(1)) 
%     while 1

        i = i + 1;
        
        %==============================================================================%
        %.. Path Following Algorithm
        
        % Step 1
        % Distance between initial waypoint and current UAV position, Ru
        Ru_vect = Wi - p(:,i); 
        Ru = norm(Wi - p(:,i));

        % Step 2
        % Orientation of vector from initial waypoint to final waypoint, theta
        theta1 = atan2(Wf(2) - Wi(2), Wf(1) - Wi(1));
        theta2 = atan2(Wf(3) - Wi(3), sqrt(  (Wf(1) - Wi(1))^2 + (Wf(2)-Wi(2))^2  ));
        
        % Step 3
        % Orientation of vector from initial waypoint to current UAV position, theta_u
        theta_u1 = atan2(p(2,i) - Wi(2), p(1,i) - Wi(1));
        theta_u2 = atan2(p(3,i) - Wi(3), sqrt(  (p(1,i) - Wi(1))^2 + (p(2,i)-Wi(2))^2  ));
        % Difference between theta and theatu, DEL_theta
        DEL_theta1 = theta1 - theta_u1;
        DEL_thata2 =  theta2 - theta_u2;
        
        % Step 4
        % Distance between initial waypoint and q, R
        if (norm(Ru_vect) ~= 0) && (norm(Rw_vect) ~= 0)
            alpha = real(acos( dot(Ru_vect, Rw_vect)/( norm(Ru_vect) * norm(Rw_vect)) ));
        else
            alpha = 0;
        end

        R = sqrt( Ru^2 - (Ru*sin(alpha))^2 );
        
        % Step 5
        % Carrot position, s = ( xt, yt )
        xt = Wi(1) + (R + delta) * cos(theta2)*cos(theta1);
        yt = Wi(2) + (R + delta) * cos(theta2)*sin(theta1);
        zt = Wi(3) + (R + delta) * sin(theta2);
        
        % Step 6
        % Desired heading angle, psi_d
        
        psi_d = atan2(yt - p(2,i), xt - p(1,i));
        % Desired pitch angle gamma_d ( CHECK AGAIN!! )
        gamma_d = atan2(zt - p(3,i), sqrt(  (xt - p(1,i))^2 + (yt - p(2,i))^2  ));

        % Wrapping up psid
        psi_d = rem(psi_d, 2*pi);
        gamma_d = rem(gamma_d, 2*pi);

        if psi_d < -pi
            psi_d = psi_d + 2*pi;
        elseif psi_d > pi
            psi_d = psi_d-2*pi;
        end

        if gamma_d < -pi
            gamma_d = gamma_d + 2*pi;
        elseif gamma_d > pi
            gamma_d = gamma_d-2*pi;
        end
        
        % Limit turning angle
        if psi_d > pi/2
            psi_d = pi/2;
        elseif psi_d < -pi/2
            psi_d = -pi/2;
        end

        % Limit pitching angle
        if gamma_d > pi/2
            gamma_d = pi/2;
        elseif gamma_d < -pi/2
            gamma_d = -pi/2;
        end

        % Step7
        % Use *current* speed v(i) instead of constant cruise va
        v_now = max(v(i), 0.1);              % singularity guard
        umax  = v_now^2 / Rmin;              % update max lateral accel

        % Guidance Yaw command, u1
        del_psi(i+1) = (psi_d - psi(i));
        u1(i) = (kappa*del_psi(i+1) + kd*(del_psi(i+1)-del_psi(i))/dt)*v_now;

        % Guidance Pitch command, u2
        del_gam(i+1) = (gamma_d - gamma(i));
        u2(i) = (kappa*del_gam(i+1) + kd*(del_gam(i+1)-del_gam(i))/dt)*v_now;

        % Limit u1
        if u1(i) > umax
            u1(i) = umax;
        elseif u1(i) < -umax
            u1(i) = - umax;
        end

        % Limit u2
        if u2(i) > umax
            u2(i) = umax;
        elseif u2(i) < -umax
            u2(i) = - umax;
        end
        %==============================================================================%

        %.. UAV Translational Dynamics  (thrust / drag / lift / gravity)
        %  Forward (along velocity vector):
        %     m * v_dot = T - D - m*g*sin(gamma)
        %  Drag    : D = 1/2 * rho * Cd * S * v^2     (opposes motion)
        %  Lift    : L = 1/2 * rho * Cl * S * v^2     (perpendicular to v;
        %             balanced implicitly by u1, u2 in coordinated flight)
        %  Thrust  : feed-forward (gravity-along-v + drag) + P feedback (V-v)
        D = 0.5 * rho * Cd * S * v_now^2;
        L = 0.5 * rho * Cl * S * v_now^2;       %#ok<NASGU> (informational)
        T_cmd = m*g_acc*sin(gamma(i)) + D + m*Kv*(V_ref - v_now);
        T     = min(max(T_cmd, 0), T_max);     % saturate to [0, T_max]
        dv    = (T - D)/m - g_acc*sin(gamma(i));

        %.. Kinematic Model of UAV  (with time-varying speed v_now)
        dx                  =               v_now * cos( gamma(i) ) * cos( psi(i) ) ;
        dy                  =               v_now * cos( gamma(i) ) * sin( psi(i) ) ;
        dz                  =               v_now * sin( gamma(i));
        dpsi                =               u1(i) / (v_now*cos(gamma(i))) ;
        dgam                =               u2(i) / v_now;

        % UAV State Update
        x(i+1)              =               x(i) + dx * dt;
        y(i+1)              =               y(i) + dy * dt;
        z(i+1)              =               z(i) + dz * dt;
        psi(i+1)            =               psi(i) + dpsi * dt ;
        gamma(i+1)          =               gamma(i) + dgam * dt;
        v(i+1)              =               max(v(i) + dv * dt, 0.1);
        vm(i+1)             =               v(i+1);
        
        % UAV Position Vector Update
        p(:,i+1)            =               [ x(i+1), y(i+1), z(i+1) ]' ;
        
        %.. Time Update
        t(i+1)              =               t(i) + dt ;
    
        psi_final = psi(i+1);
        gamma_final = gamma(i+1); 
        x_final   = x(i+1);
        y_final   = y(i+1);
        z_final   = z(i+1);

        timeSpent = timeSpent + dt;
        
        
        if animation
            ss = scatter(x(i),y(i), 'filled', 'MarkerFaceColor','blue');
            pause(0.01)
            delete(ss)
        end
        
%         if norm([x(i+1) y(i+1) z(i+1)]' - Wf) < 1
%             break;
%         end
        if i > 10000
            break;
        end
    end
    

%     subtitle({'CCA',['\delta = ' num2str(delta) '  \kappa = ' num2str(kappa)]})
end