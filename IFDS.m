function [Paths, totalLength, foundPath] = IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, L, ELM, dwdx, dwdy, mapSpan)

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
    delta_g = Param.Rg;
    k = Param.k;
    B_U = Param.B_U;
    B_L = Param.B_L;

    % Initialization
    xd = loc_final(1);
    yd = loc_final(2);
    zd = loc_final(3);

    foundPath = 0;

    dR = 5;  % m  Detection Range

    switch simMode
        case 1 % Simulate by time
%             disp("Simulating by time for " + num2str(tsim) + " s.")
            for t = 1:tsim
                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                if norm([xx yy zz] - [xd yd zd]) < targetThresh
%                     disp('Target destination reached!')
                    Wp = Wp(:,1:t);
                    Paths{L,rt} = Wp;    % Save into cell array
                    break
                else
                    % --------------- Weather constraints ------------
                    Object = create_scene(scene, Object, xx, yy, zz, rt);
                    if k~=0
                        omega = ELM(xx+1, yy+101);
                        dwdx_now = dwdx(xx+1, yy+101);
                        dwdy_now = dwdy(xx+1, yy+101);
    
                        for j = 1:Param.numObj
                            Gm = Object(j).Gamma;
                            dGdx = Object(j).n(1);
                            dGdy = Object(j).n(2);
                            dGdz = Object(j).n(3);
                            dGx_p = dGdx + k*exp( (B_L - omega)/(B_L - B_U) * log((Gm-1)/k +1) ) * ...
                                ( log((Gm-1)/k +1)/(B_L-B_U) * dwdx_now - ((B_L-omega)/((Gm-1+k)*(B_L - B_U))) *dGdx );
                            
                            dGy_p = dGdy + k*exp( (B_L - omega)/(B_L - B_U) * log((Gm-1)/k +1) ) * ...
                                ( log((Gm-1)/k+1)/(B_L-B_U) * dwdy_now - ((B_L-omega)/((Gm-1+k)*(B_L - B_U))) *dGdy );
                            
                            dGz_p = dGdz + k*exp( (B_L - omega)/(B_L - B_U) * log((Gm-1)/k +1) ) * ...
                                ( -((B_L-omega)/((Gm-1+k)*(B_L - B_U))) *dGdz );
 

                            Object(j).Gamma = Object(j).Gamma - k* (exp( (B_L - omega)/(B_L - B_U) * log((Gm-1)/k +1) ) -1);
    
                            Object(j).n = [dGx_p; dGy_p; dGz_p];
                            Object(j).t = [dGy_p; -dGx_p; 0];
                        end
                    end
    
                    [UBar, rho0, sigma0] = calc_ubar(xx, yy, zz, xd, yd, zd, ...
                        Object, rho0, sigma0, useOptimizer, delta_g, C, sf, t);

                    Wp(:,t+1) = Wp(:,t) + UBar * dt;
                end

            end

        case 2 % simulate by reaching distance
           
            t = 1;
            while true
  
                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                % if t>1000
                %     break
                % end
      
                % elevationsInRadius = extractElevations(ELM.Values, round(xx)+1, round(yy)+ mapSpan/2+1, dR);
                maxHeight = 0;
                maxPeakOrigin = [];

                % --------------Consider max peak as the object (increase computing time )----
                % for ix = xx + 1 -dR/2 : xx + 1 +dR/2
                %     for iy = yy + mapSpan/2 + 1 -dR/2 : yy + mapSpan/2 + 1 +dR/2
                %         peakHeight = ELM(ix, iy);
                %         if peakHeight > maxHeight && peakHeight > zz 
                %             maxPeakOrigin = [ix, iy, ELM(ix, iy)];
                %             maxHeight = peakHeight;
                %         end
                %     end
                % end


                % elevInRadius = ELM( (xx + 1 -dR/2 : xx + 1 +dR/2),  (yy + mapSpan/2 + 1 -dR/2 : yy + mapSpan/2 + 1 +dR/2)  );
                
                % Object = create_scene(scene, Object, xx, yy, zz, rt);
                if norm([xx yy zz] - [xd yd zd]) < targetThresh
%                     disp('Target destination reached!')
                    Wp = Wp(:,1:t);
                    Paths{L,rt} = Wp;    % Save into cell array
                    foundPath = 1;
                    break
                else
                    % --------------- Terrain constraints ------------
                    % Gm = zz - ELM(xx+1, yy+101) + 1;
                    % dGdx = dwdx(xx+1, yy+101);
                    % dGdy = dwdy(xx+1, yy+101);

                    Gm = zz - ELM(xx+1, yy+mapSpan/2+1) + 1;
                    % Gm = 100*(  exp(5*(zz- ELM(xx+1, yy+mapSpan/2+1) -1))) +1;
                    dGdx = -dwdx(xx+1, yy+mapSpan/2+1);
                    dGdy = -dwdy(xx+1, yy+mapSpan/2+1);

                    dGdz = 1;
    
                    nn = [dGdx; dGdy; dGdz];
                    tt = [dGdy; -dGdx; 0];

                    elm = struct;
                    elm.Gamma = Gm;
                    elm.n = nn;
                    elm.t = tt;
    
                    [UBar, rho0, sigma0] = calc_ubar(xx, yy, zz, xd, yd, zd, ...
                        elm, rho0, sigma0, useOptimizer, delta_g, C, sf, t, maxPeakOrigin);
                    Wp(:,t+1) = Wp(:,t) + UBar * dt;
                end
                t = t+1;
            end
            
    end

    %======================= post-Calculation =============================
    if foundPath == 1
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
    else
        totalLength = 0;
    end

end

function [UBar, rho0, sigma0]  = calc_ubar(X, Y, Z, xd, yd, zd, elm, rho0, sigma0, useOptimizer, delta_g, C, sf, time, maxPeakOrigin)

    dist = sqrt((X - xd)^2 + (Y - yd)^2 + (Z - zd)^2);

    u = -[C*(X - xd)/dist, C*(Y - yd)/dist, C*(Z - zd)/dist]';
    
    %% Pre-allocation
    numObj = size(elm,2);
    Mm = zeros(3);
    sum_w = 0;

    for j = 1:numObj

        % Reading Gamma for each object
        Gamma = elm(j).Gamma;
        
        % Unit normal vector and Unit tangential vector
        n = elm(j).n; 
        t = elm(j).t;
    
        % Object Distance from UAV
        dist_obj = 1;
        if ~isempty(maxPeakOrigin) == 1
            disp("maxPeak detected")
            x0 = maxPeakOrigin(1);
            y0 = maxPeakOrigin(2);
            z0 = maxPeakOrigin(3);

            dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);
        end

        % Modular Matrix (Perturbation Matrix
        ntu = n' * u;
        if ntu < 0 || sf == 1

            % ---- optimize the rho0, sigma0 for each object
            % if useOptimizer == 2
            %     if mod(time,5)==0 % optimize every 5 waypoints
            %         [rho0, sigma0] = path_opt2(Gamma, n, t, u, dist, dist_obj, rho0, sigma0);
            %     end
            % end
            % ---------------------------------------------------
            
            rho = rho0 * exp(1 - 1/(dist_obj * dist));
            sigma = sigma0 * exp(1 - 1/(dist_obj * dist));
            % rho = rho0 * exp(1 - 1/(dist));
            % sigma = sigma0 * exp(1 - 1/(dist));


            if norm(t) == 0
                term2 = zeros(3);
            else
                term2 = t*n'/(abs(Gamma)^(1/sigma)*norm(t)*norm(n));
            end

            M = eye(3) - n*n'/(abs(Gamma)^(1/rho)*(n')*n)...
            + term2;  % tao is removed for now
        elseif ntu >= 0 && sf == 0
            M = eye(3);
        end  

        % Weight
        w = 1;
        for i = 1:numObj
            if i == j
                continue
            else
                w = w * (elm(i).Gamma - 1)/...
                    ((elm(j).Gamma - 1) + (elm(i).Gamma - 1));
            end
        end
        sum_w = sum_w + w;

        % Saving to Field
        elm(j).n = n;
        elm(j).t = t;
        % elm(j).dist = dist_obj;
%         Obj(j).rho = rho;
%         Obj(j).sigma = sigma;
        elm(j).M  = M;
        elm(j).w = w;
    
    end
    for j = 1:numObj
        elm(j).w_tilde = elm(j).w/sum_w;
        Mm = Mm + elm(j).w_tilde * elm(j).M;
    end

    UBar = M*u;

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

function Obj = create_scene(num, Obj, X, Y, Z, rt)
    switch num
        case 0
            Obj(1) = create_ceiling(100, 0, 50, 200, 10, Obj(1));

        case 1  % Single object
%             Obj(1) = create_cone(100, 5, 0, 50, 80, Obj(1));

            % Obj(1) = create_sphere(100, 5, 0, 50, Obj(1));
            Obj(1) = create_sphere(100, 100, 0, 50, Obj(1));

    
        case 2 % 2 objects
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));

%             Obj(1) = create_cylinder(60, 100, 0, 30, 50, Obj(1));
%             Obj(2) = create_sphere(120, -100, 0, 50, Obj(2));
    
        case 3 % 3 objects
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));
            Obj(3) = create_cone(168, 0, 0, 25, 80, Obj(3));

        case 4 % single(complex) object
            Obj(1) = create_cylinder(100, 5, 0, 25, 200, Obj(1));
            Obj(2) = create_pipe(60, 20, 60, 80, 5, Obj(2));
            Obj(3) = create_pipe(130, -30, 30, 100, 50, Obj(3));
        case 5
            Obj(1) = create_cylinder(50, -20, 0, 30, 50, Obj(1));
            Obj(2) = create_cone(100, -20, 0, 30, 50, Obj(2));
            Obj(3) = create_pipe(150, -20, 0, 30, 50, Obj(3));
    
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
%             Oy = -50 + 2*single(rt);
%             Ox = 90 - 2*single(rt);
            Obj(1) = create_cylinder(100 + 50*sin(rt/8), 0 + 50*cos(rt/8), 0, 20, 80, Obj(1));
            Obj(2) = create_sphere(100, 0, 0, 30, Obj(2));
            Obj(3) = create_cylinder(100 - 50*sin(rt/8), 0 - 50*cos(rt/8), 0, 20, 50, Obj(3));

        case 42
            % Original
%             Oy1 = -5 + 60*cos(0.4*single(rt));
%             Oy2 = -20 - 20*sin(0.8*single(rt));
%             Oz2 =  60 + 20*cos(0.8*single(rt));
%             Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
%             Obj(2) = create_cylinder(110, -10, 0, 25, 80,Obj(2));
%             Obj(3) = create_cylinder(80, Oy1, 0, 20, 60, Obj(3));
%             Obj(4) = create_sphere(160, Oy2, Oz2, 30, Obj(4));

            % New
            Oy1 = -5 + 60*cos(0.4*single(rt));
            Oy2 = -20 - 20*sin(0.8*single(rt));
            Oz2 =  60 + 20*cos(0.8*single(rt));
            Obj(1) = create_cylinder(40, 5, 0, 30, 40, Obj(1));
            Obj(2) = create_cone(120, -10, 0, 25, 80,Obj(2));
            Obj(3) = create_cylinder(80, Oy1, 0, 10, 60, Obj(3));
            Obj(4) = create_sphere(160, Oy2, Oz2, 20, Obj(4));
        case 44
            Oy1 = 0 + 60*sin(0.7*single(rt));
            Oy2 = 0 + 60*cos(0.7*single(rt));
            shift = 40*sin(0.5*single(rt));
            Obj(1) = create_cylinder(40, 5, 0, 30, 80, Obj(1));
            Obj(2) = create_pipe(40, -50, 0, 50, 30, Obj(2));
            Obj(3) = create_pipe(150, 50, 0, 40, 60, Obj(3));
            Obj(4) = create_pipe(150,-10, 0, 40, 80, Obj(4));
            Obj(5) = create_pipe(110, Oy1, 0, 20, 50, Obj(5));
            Obj(6) = create_pipe(80, Oy2, 0, 30, 30, Obj(6));
            Obj(7) = create_sphere(100 + shift, 0 + shift, 60, 30, Obj(7));
    
        case 69 
            Obj(1) = create_cylinder(100, 5, 0, 30, 80, Obj(1));
            Obj(2) = create_sphere(100, 30, 0, 40, Obj(2));
            Obj(3) = create_sphere(100, -20, 0, 40, Obj(3));
            Obj(4) = create_sphere(100, 5, 80, 30, Obj(4));
        case 6969
            Obj(1) = create_sphere(100 + 30*sin(rt/8), 0 + 30*cos(rt/8), 0, 40, Obj(1));
            Obj(2) = create_cylinder(100, 0, 0, 40, 80, Obj(2));
            Obj(3) = create_sphere(100 - 30*sin(rt/8), 0 - 30*cos(rt/8), 0, 40, Obj(3));
    end

    function Obj = create_sphere(x0, y0, z0, D, Obj)
        
        a = D/2;   b = D/2;   c = D/2;      % Object's axis length
        p = 1;     q = 1;     r = 1;        % Index parameters
       
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];
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
        p = 1;     q = 1;     r = 4;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();

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

function elevationsInRadius = extractElevations(terrainMatrix, x, y, R)
    [rows, cols] = size(terrainMatrix);

    % Create a grid of coordinates
    [X, Y] = meshgrid(1:cols, 1:rows);

    % Calculate distances from the UAV location
    distances = sqrt((X - x).^2 + (Y - y).^2)


    % Create a logical mask for points within the radius
    withinRadius = distances <= R;

    % Use the mask to extract elevations
    elevationsInRadius = terrainMatrix(withinRadius);

    % Optionally, if you want to get the coordinates of the points as well
    [rowsInRadius, colsInRadius] = find(withinRadius);
    coordinatesInRadius = [rowsInRadius, colsInRadius];
end
