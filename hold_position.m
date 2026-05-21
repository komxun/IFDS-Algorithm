function [pos_hist, vm, state, filters, timeSpent, logger] = ...
         hold_position(state, filters, dt_hold, P, logger)
% HOLD_POSITION  Simulate SE(3) hover while IFDS has no path.
%
%   Commands xd = current position with zero reference velocity/accel.
%   The SE(3) geometric controller naturally regulates to hover: thrust
%   balances gravity, moments drive R to level (b1 toward psi = 0), and
%   velocity/angular-velocity errors damp to zero.
%
% Inputs :  state     current UAV state (p, v, R, Omega)
%           filters   DirtyDerivative filter handles (mutated in place)
%           dt_hold   duration of the hold                          [s]
%           P         UAV / controller parameters
%
% Outputs:  pos_hist  3xN  inertial position history (≈ constant)   [m]
%           vm        1xN  speed history (decays to 0)              [m/s]
%           state     updated state (should be near hover)
%           filters   updated filter handles
%           timeSpent simulated time elapsed                         [s]
%
% Implementation note: this is a degenerate call to SE3Track with a
% zero-length segment (Wi = Wf = current position) and V_ref = 0. The
% tracker's loop treats L_seg < 1e-6 as degenerate and exits early, so
% we replicate the inner loop here with xd held at a constant point.

Ts = P.Ts;
e3 = [0;0;1];
J  = diag([P.Jxx P.Jyy P.Jzz]);
m  = P.mass;  g = P.gravity;

% Hover target = current position, desired heading kept at zero (world +x)
xd      = state.p;
b1d     = [1; 0; 0];

N_max   = ceil(dt_hold / Ts) + 1;
pos_hist= zeros(3, N_max);
vm      = zeros(1, N_max);
pos_hist(:,1) = state.p;
vm(1)         = norm(state.v);

t = 0; k = 1;
while t < dt_hold
    % Zero reference derivatives — we want to stop
    xd_2dot = zeros(3,1);
    xd_3dot = zeros(3,1);
    xd_4dot = zeros(3,1);
    b1d_1dot = zeros(3,1);
    b1d_2dot = zeros(3,1);

    v_1dot = filters.dv1dt.calculate(state.v);
    v_2dot = filters.dv2dt.calculate(v_1dot);

    ex = state.p - xd;
    ev = state.v;                  % xd_1dot = 0
    ea = v_1dot - xd_2dot;
    ej = v_2dot - xd_3dot;

    A  = -P.kx*ex - P.kv*ev - m*g*e3 + m*xd_2dot;
    nA = norm(A);  if nA < 1e-6, nA = 1e-6; end
    f  = dot(-A, state.R*e3);

    b3c = -A / nA;
    Cv  = cross(b3c, b1d);
    nC  = norm(Cv);  if nC < 1e-6, Cv = [0;1;0]; nC = 1; end
    b1c = -(1/nC) * cross(b3c, Cv);
    b2c =  Cv / nC;
    Rc  = [b1c, b2c, b3c];

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

    Rc_1dot     = [b1c_1dot, b2c_1dot, b3c_1dot];
    Rc_2dot     = [b1c_2dot, b2c_2dot, b3c_2dot];
    Omegac      = vee(Rc' * Rc_1dot);
    Omegac_1dot = vee(Rc' * Rc_2dot - hat(Omegac) * hat(Omegac));

    eR     = 0.5 * vee(Rc'*state.R - state.R'*Rc);
    eOmega = state.Omega - state.R'*Rc*Omegac;

    M = -P.kR*eR - P.kOmega*eOmega + cross(state.Omega, J*state.Omega) ...
      - J*(hat(state.Omega)*state.R'*Rc*Omegac - state.R'*Rc*Omegac_1dot);

    % Telemetry log (same format as SE3Track)
    if ~isempty(logger.t), t_global = logger.t(end) + Ts; else, t_global = 0; end
    deltaF = P.Mix * [f; M];
    Psi    = 0.5 * trace(eye(3) - Rc'*state.R);
    logger.t(:,end+1)      = t_global;
    logger.x(:,end+1)      = state.p;
    logger.xd(:,end+1)     = xd;
    logger.v(:,end+1)      = state.v;
    logger.vd(:,end+1)     = zeros(3,1);
    logger.Omega(:,end+1)  = state.Omega;
    logger.Omegac(:,end+1) = Omegac;
    logger.Psi(:,end+1)    = Psi;
    logger.f(:,end+1)      = f;
    logger.M(:,end+1)      = M;
    logger.deltaF(:,end+1) = deltaF;

    % RK4 step (same as SE3Track; zero-order hold on f, M)
    k1 = deriv(state, f, M, m, g, J);
    s2 = add_state(state, k1, Ts/2);  k2 = deriv(s2, f, M, m, g, J);
    s3 = add_state(state, k2, Ts/2);  k3 = deriv(s3, f, M, m, g, J);
    s4 = add_state(state, k3, Ts);    k4 = deriv(s4, f, M, m, g, J);
    kavg.p     = (k1.p     + 2*k2.p     + 2*k3.p     + k4.p)    /6;
    kavg.v     = (k1.v     + 2*k2.v     + 2*k3.v     + k4.v)    /6;
    kavg.R     = (k1.R     + 2*k2.R     + 2*k3.R     + k4.R)    /6;
    kavg.Omega = (k1.Omega + 2*k2.Omega + 2*k3.Omega + k4.Omega)/6;
    state      = add_state(state, kavg, Ts);
    state.R    = proj_SO3(state.R);

    t = t + Ts;  k = k + 1;
    pos_hist(:,k) = state.p;
    vm(k)         = norm(state.v);
end

pos_hist  = pos_hist(:, 1:k);
vm        = vm(1:k);
timeSpent = t;
end

%% -- Local helpers (duplicated from SE3Track to keep file self-contained)

function k = deriv(s, f, M, m, g, J)
    e3 = [0;0;1];
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
    [U, ~, V] = svd(R);
    R = U * V';
    if det(R) < 0, R = U * diag([1,1,-1]) * V'; end
end
