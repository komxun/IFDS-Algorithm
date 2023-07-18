function [Paths, Object, totalLength] = IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, L, Object, weatherMat, dwdx, dwdy)

    % Read the parameters
    simMode = Param.simMode;
    scene =  Param.scene;
    sf = Param.sf;
    targetThresh = Param.targetThresh;
    tsim = Param.tsim;
    dt = Param.dt;
    C = Param.C;
    showDisp = Param.showDisp;
    useOptimizer = Param.useOptimizer;
    Rg = Param.Rg;

    % Initialization
    xd = loc_final(1);
    yd = loc_final(2);
    zd = loc_final(3);

    switch simMode
        case 1 % Simulate by time
%             disp("Simulating by time for " + num2str(tsim) + " s.")
            for t = 1:tsim
                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                % --------------- Weather constraints ------------
        
                if (xx < 199) && (yy < 199)
                    % Data Interpolating Method
                    option = 'nearest';
                    omega = interp2(weatherMat(:,:,rt), xx+1, yy+101, option);
                    dwdx_now = interp2(dwdx, xx+1, yy+101, option);
                    dwdy_now = interp2(dwdy, xx+1, yy+101, option);
                else
                    % Data Cutting method
                    xid = min(round(xx)+1,200);
                    yid = min(round(yy)+100+1,200);
                    omega = weatherMat(yid, xid, rt);
                    dwdx_now = dwdx(yid, xid);
                    dwdy_now = dwdy(yid, xid);
                end
                
                Object = create_scene(scene, Object, xx, yy, zz, rt, omega, dwdx_now, dwdy_now);

                [UBar, rho0, sigma0] = calc_ubar(xx, yy, zz, xd, yd, zd, ...
                    Object, rho0, sigma0, useOptimizer, Rg, C, sf, t);
                
                if norm([xx yy zz] - [xd yd zd]) < targetThresh
%                     disp('Target destination reached!')
                    Wp = Wp(:,1:t);
                    Paths{L,rt} = Wp;    % Save into cell array
                    break
                else
                    Wp(:,t+1) = Wp(:,t) + UBar * dt;
                end

            end

        case 2 % simulate by reaching distance
            disp("Simulating by distance until " + num2str(targetThresh) + " m error range")
            t = 1;
   
            while norm([Wp(1,t) Wp(2,t) Wp(3,t)] - double([xd yd zd])) > targetThresh

                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                if n(xx,yy,zz)'*u(xx,yy,zz) < 0 || sf == 1
        %             disp('case 1 activated')
                    Wp(:,t+1) = Wp(:,t) + double(UBar(Wp(1,t), Wp(2,t), Wp(3,t))) * dt;
        
                elseif n(xx,yy,zz)'*u(xx,yy,zz) >= 0 && sf == 0
        %             disp('case 2 activated')
                    Wp(:,t+1) = Wp(:,t) + double(u(Wp(1,t), Wp(2,t), Wp(3,t))) * dt;
                end  
                t = t+1;
            end
            % Removing extra space
            Wp = Wp(:,1:t,:); 
            disp('Target destination reached!')
    end

    %======================= post-Calculation =============================
    waypoints = Paths{1,1}';         % Calculate pairwise distances between waypoints
    differences = diff(waypoints);   % the differences between consecutive waypoints
    squaredDistances = sum(differences.^2, 2); 
    
    % Calculate the total path length
    totalLength = sum(sqrt(squaredDistances));
    
    % Display the total path length
    if showDisp
        fprintf('Total path length: %.2f m\n', totalLength);
        fprintf('Total flight time: %.2f s\n', totalLength/C);
    end

end

function [UBar, rho0, sigma0]  = calc_ubar(X, Y, Z, xd, yd, zd, Obj, rho0, sigma0, useOptimizer, Rg, C, sf, time)

    dist = sqrt((X - xd)^2 + (Y - yd)^2 + (Z - zd)^2);

    u = -[C*(X - xd)/dist, C*(Y - yd)/dist, C*(Z - zd)/dist]';
    
    %% Pre-allocation
    numObj = size(Obj,2);
    Mm = zeros(3);
    sum_w = 0;

    for j = 1:numObj

        % Reading Gamma for each object
        Gamma = Obj(j).Gamma;
        
        % Unit normal vector and Unit tangential vector
        n = Obj(j).n; 
        t = Obj(j).t;
    
        % Object Distance from UAV
        x0 = Obj(j).origin(1);
        y0 = Obj(j).origin(2);
        z0 = Obj(j).origin(3);

        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);

        % Modular Matrix (Perturbation Matrix
        ntu = n' * u;
        if ntu < 0 || sf == 1
            % ---- optimize the rho0, sigma0 for each object
            if useOptimizer == 2
                if mod(time,5)==0 % optimize every 5 waypoints
                    [rho0, sigma0] = path_opt2(Gamma, n, t, u, dist, dist_obj, rho0, sigma0);
                end
            end
            % ---------------------------------------------------
            
            if useOptimizer == 0
                % Add Gap Constraint
                Rstar = Obj(j).Rstar;
                rho0_star = log(abs(Gamma))/(log(abs(Gamma - ((Rstar + Rg)/Rstar)^2 + 1))) * rho0;
                rho = rho0_star * exp(1 - 1/(dist_obj * dist));
            else
                % Without SafeGuard
                rho = rho0 * exp(1 - 1/(dist_obj * dist));
            end

            sigma = sigma0 * exp(1 - 1/(dist_obj * dist));

            M = eye(3) - n*n'/(abs(Gamma)^(1/rho)*(n')*n)...
            + t*n'/(abs(Gamma)^(1/sigma)*norm(t)*norm(n));  % tao is removed for now
        elseif ntu >= 0 && sf == 0
            M = eye(3);
        end  

        % Weight
        w = 1;
        for i = 1:numObj
            if i == j
                continue
            else
                w = w * (Obj(i).Gamma - 1)/...
                    ((Obj(j).Gamma - 1) + (Obj(i).Gamma - 1));
            end
        end
        sum_w = sum_w + w;

        % Saving to Field
        Obj(j).n = n;
        Obj(j).t = t;
        Obj(j).dist = dist_obj;
%         Obj(j).rho = rho;
%         Obj(j).sigma = sigma;
        Obj(j).M  = M;
        Obj(j).w = w;
    
    end

    for j = 1:numObj
        Obj(j).w_tilde = Obj(j).w/sum_w;
        Mm = Mm + Obj(j).w_tilde * Obj(j).M;
    end

    UBar = Mm*u;

    function [rho0, sigma0] = path_opt2(Gamma, n, t, u, dist, dist_obj, rho0, sigma0)
    
        xg = [rho0; sigma0];

        lower_bound_rho = 0.05;  % <0.05 issue started to occur
        lower_bound_sigma = 0;
        
        upper_bound_rho = 2;
        upper_bound_sigma = 1;
        
        % Set up the optimization problem
        problem.objective = @(x) norm_ubar(x(1), x(2), Gamma, n, t, u, dist, dist_obj);
        problem.x0 = xg;
        problem.Aineq = [];
        problem.bineq = [];
        problem.Aeq = [];
        problem.beq = [];
        problem.lb = [lower_bound_rho; lower_bound_sigma];
        problem.ub = [upper_bound_rho; upper_bound_sigma];
        problem.nonlcon = [];
        problem.solver = 'fmincon';  % specify the solver
        problem.options = optimoptions('fmincon', ...
            'Algorithm', 'interior-point', ...   % option2: sqp
            'Display', 'off');
        
        % Call fmincon
        [xOpt, fval, exitflag, output] = fmincon(problem);
%         disp(output)
        rho0 = xOpt(1);
        sigma0 = xOpt(2);

    end

end

function Obj = create_scene(num, Obj, X, Y, Z, rt, omega, dwdx, dwdy)
    switch num
        case 0  % Single object
%             Obj(1) = create_cone(100, 5, 0, 50, 80, Obj(1));
            Obj(1) = create_sphere(100, 5, 0, 50, Obj(1));
%             Obj(1) = create_sphere(100, 80, 0, 50, Obj(1));
%             Obj(1) = create_cylinder(100, 5, 0, 25, 60, Obj(1));
%             Obj(1) = create_pipe(100, 5, 0, 25, 60, Obj(1));
            
        case 1 % single(complex) object
            Obj(1) = create_cylinder(100, 5, 0, 25, 200, Obj(1));
            Obj(2) = create_pipe(60, 20, 60, 80, 5, Obj(2));
            Obj(3) = create_pipe(130, -30, 30, 100, 5, Obj(3));
    
        case 2 % 2 objects
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));
    
        case 3 % 3 objects
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));
            Obj(3) = create_pipe(168, 0, 0, 25, 80, Obj(3));
    
        case 12 % 12 objects
            Obj(1) = create_cylinder(100, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_pipe(140, 20, 0, 40,10, Obj(2));
            Obj(3) = create_pipe(20, 20, 0, 24, 40, Obj(3));
            Obj(4) = create_pipe(55, -20, 0, 28, 50, Obj(4));
            Obj(5) = create_sphere(53, -60, 0, 50, Obj(5));
            Obj(6) = create_pipe(150, -80, 0, 40, 50, Obj(6));
            Obj(7) = create_cone(100, -35, 0, 50,45, Obj(7));
            Obj(8) = create_cone(170, 2, 0, 20,50, Obj(8));
            Obj(9) = create_cone(60, 35, 0, 50,30, Obj(9));
            Obj(10) = create_cylinder(110, 70, 0, 60, 50, Obj(10));
            Obj(11) = create_pipe(170, 60, 0, 40, 27, Obj(11));
            Obj(12) = create_cone(150, -30, 0, 32, 45, Obj(12));
        case 7 % 7 objects
            Obj(1) = create_cone(60,8, 0, 70, 50, Obj(1));
            Obj(2) = create_cone(100,-24, 0, 89, 100, Obj(2));
            Obj(3) = create_cone(160,40, -4, 100, 30, Obj(3));
            Obj(4) = create_cone(100,100, -10, 150, 100, Obj(4));
            Obj(5) = create_cone(180,-70, -10, 150, 20, Obj(5));
            Obj(6) = create_cone(75,-75, -10, 150, 40, Obj(6));
            Obj(7) = create_cylinder(170, -6, 0, 34, 100, Obj(7));
        case 41
            Oy = -80 + 2*single(rt);
            Obj(1) = create_cylinder(100, Oy, 0, 50, 80, Obj(1));
        case 42
            Oy1 = 0 + 50*sin(0.2*single(rt));
            Oy2 = -20 - 20*sin(0.5*single(rt));
            Oz2 =  40 + 20*cos(0.5*single(rt));
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_cylinder(110, -10, 0, 25, 80,Obj(2));
            Obj(3) = create_cylinder(80, Oy1, 0, 20, 60, Obj(3));
            Obj(4) = create_sphere(160, Oy2, Oz2, 30, Obj(4));
    
        case 69 %pp
            Obj(1) = create_cylinder(100, 5, 0, 30, 80, Obj(1));
            Obj(2) = create_sphere(100, 30, 0, 40, Obj(2));
            Obj(3) = create_sphere(100, -20, 0, 40, Obj(3));
            Obj(4) = create_sphere(100, 5, 80, 30, Obj(4));
    end

    function Obj = create_sphere(x0, y0, z0, D, Obj)
        
        a = D/2;   b = D/2;   c = D/2;      % Object's axis length
        p = 1;     q = 1;     r = 1;        % Index parameters
       
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();

        k = 1000;
        B_u = 1;     % Good: k=1 | B_u=0.7 | B_L = 0 
        B_L = 0.2;   % Good: k=100 | B_u=1 | B_L = 0.5
        
        % V4
        if omega<= B_L
            omega = B_L;
        end
        if k~=0
            dGdx_p = dGdx + k*exp( (B_L - omega)/(B_L - B_u) * log((Gamma-1)/k +1) ) * ...
                ( log((Gamma-1)/k +1)/(B_L-B_u) * dwdx - ((B_L-omega)/((Gamma-1+k)*(B_L - B_u))) *dGdx );
            
            dGdy_p = dGdy + k*exp( (B_L - omega)/(B_L - B_u) * log((Gamma-1)/k +1) ) * ...
                ( log((Gamma-1)/k+1)/(B_L-B_u) * dwdy - ((B_L-omega)/((Gamma-1+k)*(B_L - B_u))) *dGdy );
            
            dGdz_p = dGdz + k*exp( (B_L - omega)/(B_L - B_u) * log((Gamma-1)/k +1) ) * ...
                ( -((B_L-omega)/((Gamma-1+k)*(B_L - B_u))) *dGdz );

            Gamma = Gamma - k* (exp( (B_L - omega)/(B_L - B_u) * log((Gamma-1)/k +1) ) -1);

            n = [dGdx_p; dGdy_p; dGdz_p];
            t = [dGdy_p; -dGdx_p; 0];
        else
            n = [dGdx; dGdy; dGdz];
            t = [dGdy; -dGdx; 0];
        end
%-----------------------------------------------------------------------
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end

    function Obj = create_cylinder(x0, y0, z0, D, h, Obj)
    
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 5;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();

        dGdx_p = dGdx - exp(omega * log(abs(Gamma)))*(log(abs(Gamma))*dwdx + (omega/Gamma)*dGdx);
        dGdy_p = dGdy - exp(omega * log(abs(Gamma)))*(log(abs(Gamma))*dwdy + (omega/Gamma)*dGdy);
        dGdz_p = dGdz - exp(omega * log(abs(Gamma)))*((omega/Gamma)*dGdz);

        Gamma = 1+ Gamma - exp(omega * log(Gamma));

        % n and t
%         n = [dGdx; dGdy; dGdz];
%         t = [dGdy; -dGdx; 0];
        
        n = [dGdx_p; dGdy_p; dGdz_p];
        t = [dGdy_p; -dGdx_p; 0];

        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0]; 
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end    
    end
    
    function Obj = create_cone(x0, y0, z0, D, h, Obj)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 0.5;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % n and t
        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];

        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end    
    end
     
    function Obj = create_pipe(x0, y0, z0, D, h, Obj)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 2;     q = 2;     r = 2;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();

        % n and t
        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r; 
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end

    
end
