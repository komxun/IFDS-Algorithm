classdef DirtyDerivative < handle
    %DIRTYDERIVATIVE Band-limited filtered derivative: P(s) = s / (tau*s + 1).
    %   Provides nth-order causal derivatives of a sampled signal. The
    %   first (order - 1) outputs are held at zero so a fresh instance
    %   does not produce spurious transients.
    %
    %   Usage:
    %       d = DirtyDerivative(order, tau, Ts);
    %       xdot = d.calculate(x);           % call once per step
    %
    %   Source: se3quad/matlab/DirtyDerivative.m
    properties (Access = public)
        tau   = 0.05;
        Ts    = 0.01;
        a1;
        a2;
        order;
        dot;
        x_d1;
        it    = 1;
    end
    methods
        function dxdt = DirtyDerivative(order, tau, Ts)
            dxdt.a1    = (2*tau - Ts) / (2*tau + Ts);
            dxdt.a2    = 2 / (2*tau + Ts);
            dxdt.order = order;
            dxdt.tau   = tau;
            dxdt.Ts    = Ts;
        end
        function xdot = calculate(dxdt, x)
            x = x(:);
            if dxdt.it == 1
                dxdt.dot  = zeros(size(x));
                dxdt.x_d1 = zeros(size(x));
            end
            if dxdt.it > dxdt.order
                dxdt.dot = dxdt.a1*dxdt.dot + dxdt.a2*(x - dxdt.x_d1);
            end
            dxdt.it   = dxdt.it + 1;
            dxdt.x_d1 = x;
            xdot = dxdt.dot;
        end
    end
end
