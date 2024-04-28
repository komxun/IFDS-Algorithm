function [Paths, Object, totalLength, foundPath] = IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, L, Object, weatherMat, dwdx, dwdy)

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

    switch simMode
        case 1 % Simulate by time

            for t = 1:tsim
                Wp(:,t) = real(Wp(:,t));
                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                if t>1000
                    break
                end
                Object = create_scene(scene, Object, xx, yy, zz, rt);
                if norm([xx yy zz] - [xd yd zd]) < targetThresh
%                     disp('Target destination reached!')
                    Wp = Wp(:,1:t);
                    Paths{L,rt} = Wp;    % Save into cell array
                    foundPath = 1;
                    break
                else
                    % --------------- Weather constraints ------------
                    if k~=0
                        omega = weatherMat(xx+1, yy+101);
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
    
                    [UBar, rho0, sigma0, errFlag] = calc_ubar(xx, yy, zz, xd, yd, zd, ...
                        Object, rho0, sigma0, useOptimizer, delta_g, C, sf, t);
                    if errFlag == 1
                        disp("errFlag")
                        break
                    end
                    Wp(:,t+1) = Wp(:,t) + UBar * dt;
                end
%                 t = t+1;
            end
            Wp = Wp(:,1:t);
            Paths{L,rt} = Wp;    % Save into cell array
            if errFlag == 0 %&& all([Object(:).Gamma] > 0.5)
                foundPath = 1;
            else
                foundPath = 0;
            end

        case 2 % simulate by reaching distance
           
            t = 1;
            while true
                xx = Wp(1,t);
                yy = Wp(2,t);
                zz = Wp(3,t);

                if t>1000
                    break
                end
                Object = create_scene(scene, Object, xx, yy, zz, rt);
                if norm([xx yy zz] - [xd yd zd]) < targetThresh
%                     disp('Target destination reached!')
                    Wp = Wp(:,1:t);
                    Paths{L,rt} = Wp;    % Save into cell array
                    foundPath = 1;
                    break
                else
                    % --------------- Weather constraints ------------
                    if k~=0
                        omega = weatherMat(xx+1, yy+101);
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
    
                    [UBar, rho0, sigma0, errFlag] = calc_ubar(xx, yy, zz, xd, yd, zd, ...
                        Object, rho0, sigma0, useOptimizer, delta_g, C, sf, t);
                    if errFlag == 1
                        disp("errFlag!")
                        break
                    end
                    Wp(:,t+1) = Wp(:,t) + UBar * dt;
                end
                t = t+1;
            end
            Wp = Wp(:,1:t);
            Paths{L,rt} = Wp;    % Save into cell array
            if errFlag == 0
                foundPath = 1;
            else
                foundPath = 0;
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

function [UBar, rho0, sigma0, errFlag]  = calc_ubar(X, Y, Z, xd, yd, zd, Obj, rho0, sigma0, useOptimizer, delta_g, C, sf, time)

    dist = sqrt((X - xd)^2 + (Y - yd)^2 + (Z - zd)^2);

    u = -[C*(X - xd)/dist, C*(Y - yd)/dist, C*(Z - zd)/dist]';
    
    %% Pre-allocation
    numObj = size(Obj,2);
    Mm = zeros(3);
    sum_w = 0;
    errFlag = 0;

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
                    [rho0, sigma0] = path_opt2(Gamma, n, t, u, dist, dist_obj, rho0, sigma0)
                end
            end
            % ---------------------------------------------------
            
%             if useOptimizer == 0
                % Add Gap Constraint
                Rstar = Obj(j).Rstar;
                rho0_star = log(abs(Gamma))/(log(abs(Gamma - ((Rstar + delta_g)/Rstar)^2 + 1))) * rho0;
                rho = rho0_star * exp(1 - 1/(dist_obj * dist));
%             else
%                 % Without SafeGuard
%                 rho = rho0 * exp(1 - 1/(dist_obj * dist));
%             end

            sigma = sigma0 * exp(1 - 1/(dist_obj * dist));

            M = eye(3) - n*n'/(abs(Gamma)^(1/rho)*(n')*n)...
            + t*n'/(abs(Gamma)^(1/sigma)*norm(t)*norm(n));  % tao is removed for now
        elseif ntu >= 0 && sf == 0
            M = eye(3);
        else
            errFlag = 1;
            UBar = u;
            return
            break
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



function willCollideIdx = CheckCollide(x,y,z, Object, sh_old, sv_old, r_old, rt)

    willCollideIdx = zeros(1,size(Object,2));
    for j = 1:size(Object, 2)
        xo = Object(j).origin(rt, 1);
        yo = Object(j).origin(rt, 2);
        zo = Object(j).origin(rt, 3);

        sigma_h = atan2((yo - y)/(xo - x));
        sigma_v = atan2((zo - z) / ( sqrt((xo-x)^2 + (yo-y)^2)  ));

        if (sigma_h == sh_old) && (sigma_v == sv_old) && (sign(r - r_old) == -1) 
            willCollideIdx(j) = 1; % will collide
        end
    end

end
