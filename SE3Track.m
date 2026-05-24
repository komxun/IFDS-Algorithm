function [pos_hist, vm, state, filters, timeSpent, logger] = ...
         SE3Track(Wi, Wf, state, filters, V_ref, dt_max, P, logger)
% SE3TRACK  SE(3) geometric tracker for a single IFDS path segment.
%
%   Ports the Lee et al. (2010/2011) geometric controller and full
%   rigid-body quadrotor dynamics from se3quad/matlab into the segmented
%   IFDS path-following framework (replaces CCA3D_2).
%
% Inputs :  Wi, Wf    3x1  segment endpoints (inertial frame)       [m]
%           state     struct with fields:
%                       p     3x1  inertial position               [m]
%                       v     3x1  inertial velocity               [m/s]
%                       R     3x3  body-to-inertial rotation matrix
%                       Omega 3x1  body-frame angular velocity     [rad/s]
%           filters   struct of DirtyDerivative handles (created by
%                     SE3Track_init); mutated in place.
%           V_ref     cruise reference speed along segment          [m/s]
%           dt_max    maximum segment duration (time budget)        [s]
%           P         UAV / controller parameter struct
%
% Outputs:  pos_hist  3xN  inertial positions visited this segment  [m]
%           vm        1xN  speed history  ||v||                     [m/s]
%           state     updated state struct (handed to next segment)
%           filters   updated filter handles
%           timeSpent simulated time elapsed in this segment         [s]
%
% Segment exit condition:
%   (a) UAV crosses the plane through Wf normal to the segment direction
%   (b) Reference point xd(t) has reached Wf  (|xd - Wi| >= |Wf - Wi|)
%   (c) dt_max elapsed (time budget for this main.m outer iteration)

Ts = P.Ts;                                % controller step [s]

%% Segment geometry
d_vec  = Wf - Wi;
L_seg  = norm(d_vec);
if L_seg < 1e-6
    % Degenerate segment — nothing to do.
    pos_hist  = state.p;
    vm        = norm(state.v);
    timeSpent = 0;
    return
end
d_hat   = d_vec / L_seg;
psi_d   = atan2(d_hat(2), d_hat(1));      % horizontal heading [rad]
b1d     = [cos(psi_d); sin(psi_d); 0];    % desired body-1 axis (const on seg)

e3 = [0; 0; 1];
J  = diag([P.Jxx P.Jyy P.Jzz]);
m  = P.mass;  g = P.gravity;

%% Project current drone position onto segment so the reference starts
%  at the nearest point on the segment, not always at Wi.
s0 = max(0, min(dot(state.p - Wi, d_hat), L_seg));

%% History buffers (pre-allocate conservatively)
N_max    = ceil(dt_max / Ts) + 1;
pos_hist = zeros(3, N_max);
vm       = zeros(1, N_max);
pos_hist(:,1) = state.p;
vm(1)         = norm(state.v);

%% Integration loop
t  = 0;
k  = 1;
while t < dt_max
    %% --- Reference trajectory (linear along segment at V_ref) --------
    %  xd is capped at Wf so the carrot doesn't overshoot in position,
    %  but xd_1dot is always V_ref*d_hat so the drone flies THROUGH
    %  each waypoint at cruise speed instead of braking to a stop.
    s       = V_ref * t + s0;                     % arclength (uncapped) [m]
    xd      = Wi + min(s, L_seg) * d_hat;         % position: cap at Wf
    xd_1dot = V_ref * d_hat;                      % velocity: always forward
    xd_2dot = zeros(3,1);
    xd_3dot = zeros(3,1);
    xd_4dot = zeros(3,1);
    b1d_1dot = zeros(3,1);
    b1d_2dot = zeros(3,1);

    %% --- Controller (Lee 2010/2011 geometric tracking on SE(3)) -----
    %  Dirty-derivative filters on v provide acceleration / jerk of the
    %  actual state (feed-forward compensation, Lee 2011 Eq. Appendix F).
    v_1dot = filters.dv1dt.calculate(state.v);
    v_2dot = filters.dv2dt.calculate(v_1dot);

    ex = state.p - xd;
    ev = state.v - xd_1dot;
    ea = v_1dot   - xd_2dot;
    ej = v_2dot   - xd_3dot;

    % Thrust direction / magnitude, Lee Eq. 19
    A  = -P.kx*ex - P.kv*ev - m*g*e3 + m*xd_2dot;
    nA = norm(A);
    if nA < 1e-6, nA = 1e-6; end
    f  = dot(-A, state.R*e3);                     % scalar thrust [N]

    % Desired attitude Rc
    b3c = -A / nA;
    Cv  = cross(b3c, b1d);
    nC  = norm(Cv);
    if nC < 1e-6, Cv = [0;1;0]; nC = 1; end
    b1c = -(1/nC) * cross(b3c, Cv);
    b2c =  Cv / nC;
    Rc  = [b1c, b2c, b3c];

    % Time derivatives of Rc (Lee 2011 Appendix F)
    A_1dot   = -P.kx*ev - P.kv*ea + m*xd_3dot;
    b3c_1dot = -A_1dot/nA + (dot(A, A_1dot)/nA^3)*A;
    C_1dot   = cross(b3c_1dot, b1d) + cross(b3c, b1d_1dot);
    b2c_1dot = C_1dot/nC - (dot(Cv, C_1dot)/nC^3)*Cv;
    b1c_1dot = cross(b2c_1dot, b3c) + cross(b2c, b3c_1dot);

    A_2dot   = -P.kx*ea - P.kv*ej + m*xd_4dot;
    b3c_2dot = -A_2dot/nA + (2/nA^3)*dot(A, A_1dot)*A_1dot ...
             + ((norm(A_1dot)^2 + dot(A, A_2dot))/nA^3)*A  ...
             - (3/nA^5)*(dot(A, A_1dot)^2)*A;
    C_2dot   = cross(b3c_2dot, b1d) + cross(b3c, b1d_2dot) ...
             + 2*cross(b3c_1dot, b1d_1dot);
    b2c_2dot = C_2dot/nC - (2/nC^3)*dot(Cv, C_1dot)*C_1dot  ...
             - ((norm(C_1dot)^2 + dot(Cv, C_2dot))/nC^3)*Cv ...
             + (3/nC^5)*(dot(Cv, C_1dot)^2)*Cv;
    b1c_2dot = cross(b2c_2dot, b3c) + cross(b2c, b3c_2dot)  ...
             + 2*cross(b2c_1dot, b3c_1dot);

    Rc_1dot      = [b1c_1dot, b2c_1dot, b3c_1dot];
    Rc_2dot      = [b1c_2dot, b2c_2dot, b3c_2dot];
    Omegac       = vee(Rc' * Rc_1dot);
    Omegac_1dot  = vee(Rc' * Rc_2dot - hat(Omegac) * hat(Omegac));

    % Attitude errors (Lee 2010 Eq. 10-11)
    eR     = 0.5 * vee(Rc'*state.R - state.R'*Rc);
    eOmega = state.Omega - state.R'*Rc*Omegac;

    % Moment control (Lee 2010 Eq. 13)
    M = -P.kR*eR - P.kOmega*eOmega + cross(state.Omega, J*state.Omega) ...
      - J*(hat(state.Omega)*state.R'*Rc*Omegac - state.R'*Rc*Omegac_1dot);

    %% --- Telemetry log (pre-integration: current state + controls) --
    if ~isempty(logger.t), t_global = logger.t(end) + Ts; else, t_global = 0; end
    deltaF = P.Mix * [f; M];
    Psi    = 0.5 * trace(eye(3) - Rc'*state.R);
    logger.t(:,end+1)      = t_global;
    logger.x(:,end+1)      = state.p;
    logger.xd(:,end+1)     = xd;
    logger.v(:,end+1)      = state.v;
    logger.vd(:,end+1)     = xd_1dot;
    logger.Omega(:,end+1)  = state.Omega;
    logger.Omegac(:,end+1) = Omegac;
    logger.Psi(:,end+1)    = Psi;
    logger.f(:,end+1)      = f;
    logger.M(:,end+1)      = M;
    logger.deltaF(:,end+1) = deltaF;

    %% --- RK4 integration of rigid-body dynamics (Lee 2011 Eq. 2-5) --
    %  x_dot   = v
    %  v_dot   = g*e3 - (f/m)*R*e3
    %  R_dot   = R * hat(Omega)
    %  Ome_dot = J^{-1} * (M - Ome x J*Ome)
    state = rk4_step(state, f, M, Ts, m, g, J);
    state.R = proj_SO3(state.R);                  % prevent drift

    %% --- Bookkeeping ------------------------------------------------
    t = t + Ts;
    k = k + 1;
    pos_hist(:,k) = state.p;
    vm(k)         = norm(state.v);

    %% --- Segment-done check (normal plane through Wf) ---------------
    if dot(d_vec, state.p - Wf) >= 0
        break
    end
end

%% Trim history buffers
pos_hist  = pos_hist(:, 1:k);
vm        = vm(1:k);
timeSpent = t;

end

%% =====================================================================
%% Local helpers
%% =====================================================================

function state = rk4_step(state, f, M, dt, m, g, J)
    % One RK4 step on the 18-state SE(3) rigid-body EOM.
    % Controls (f, M) are held constant over the step (zero-order hold).
    k1 = deriv(state, f, M, m, g, J);

    s2 = add_state(state, k1, dt/2);
    k2 = deriv(s2, f, M, m, g, J);

    s3 = add_state(state, k2, dt/2);
    k3 = deriv(s3, f, M, m, g, J);

    s4 = add_state(state, k3, dt);
    k4 = deriv(s4, f, M, m, g, J);

    kavg.p     = (k1.p     + 2*k2.p     + 2*k3.p     + k4.p)    /6;
    kavg.v     = (k1.v     + 2*k2.v     + 2*k3.v     + k4.v)    /6;
    kavg.R     = (k1.R     + 2*k2.R     + 2*k3.R     + k4.R)    /6;
    kavg.Omega = (k1.Omega + 2*k2.Omega + 2*k3.Omega + k4.Omega)/6;
    state      = add_state(state, kavg, dt);
end

function k = deriv(s, f, M, m, g, J)
    e3      = [0;0;1];
    k.p     = s.v;
    k.v     = g*e3 - (f/m)*(s.R*e3);
    k.R     = s.R * hat(s.Omega);
    k.Omega = J \ (M - cross(s.Omega, J*s.Omega));
end

function s_out = add_state(s, ds, dt)
    s_out.p     = s.p     + dt*ds.p;
    s_out.v     = s.v     + dt*ds.v;
    s_out.R     = s.R     + dt*ds.R;
    s_out.Omega = s.Omega + dt*ds.Omega;
end

function R = proj_SO3(R)
    % SVD projection to SO(3): corrects numerical drift in the rotation
    % matrix so R'*R = I and det(R) = +1.
    [U, ~, V] = svd(R);
    R = U * V';
    if det(R) < 0
        R = U * diag([1, 1, -1]) * V';
    end
end
