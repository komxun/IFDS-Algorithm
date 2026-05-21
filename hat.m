function ss = hat(vec)
% HAT  Map R^3 to so(3) (skew-symmetric matrix).
%   Source: github.com/justinthomas/MATLAB-tools/blob/master/hat.m
switch length(vec)
    case 3
        ss = [ 0      -vec(3)  vec(2);
               vec(3)  0      -vec(1);
              -vec(2)  vec(1)  0    ];
    case 1
        ss = [0  vec; -vec 0];
end
end
