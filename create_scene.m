function Obj = create_scene(num, Obj, X, Y, Z, rt)

    switch num
        case 0
            Obj(1) = create_ceiling(100, 0, 50, 200, 10, Obj(1));

        case 1  % Single object
%             Obj(1) = create_cone(100, 5, 0, 50, 80, Obj(1));

            Obj(1) = create_sphere(100, 5, 0, 50, Obj(1));
%             Obj(1) = create_sphere(100, 75, 0, 50, Obj(1));

    
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

        sigma_h = atan2((y0 - Y),(x0 - X));
        sigma_v = atan2((z0 - Z) , ( sqrt((x0-X)^2 + (y0-Y)^2)  ));
        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);
        Obj.sigma_h(rt) = sigma_h;
        Obj.sigma_v(rt) = sigma_v;
        Obj.dist_obj(rt) = dist_obj;
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

        sigma_h = atan2((y0 - Y),(x0 - X));
        sigma_v = atan2((z0 - Z) , ( sqrt((x0-X)^2 + (y0-Y)^2)  ));
        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);
        Obj.sigma_h(rt) = sigma_h;
        Obj.sigma_v(rt) = sigma_v;
        Obj.dist_obj(rt) = dist_obj;
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

        sigma_h = atan2((y0 - Y),(x0 - X));
        sigma_v = atan2((z0 - Z) , ( sqrt((x0-X)^2 + (y0-Y)^2)  ));
        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);
        Obj.sigma_h(rt) = sigma_h;
        Obj.sigma_v(rt) = sigma_v;
        Obj.dist_obj(rt) = dist_obj;
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

        sigma_h = atan2((y0 - Y),(x0 - X));
        sigma_v = atan2((z0 - Z) , ( sqrt((x0-X)^2 + (y0-Y)^2)  ));
        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);
        Obj.sigma_h(rt) = sigma_h;
        Obj.sigma_v(rt) = sigma_v;
        Obj.dist_obj(rt) = dist_obj;
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end


end