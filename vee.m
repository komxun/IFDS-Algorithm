function vec = vee(ss)
% VEE  Inverse of hat: map so(3) to R^3.
%   Source: github.com/justinthomas/MATLAB-tools/blob/master/vee.m
if isa(ss, 'sym')
    ss = expand(simplify(ss));
end
switch numel(ss)
    case 4
        vec = ss(1,2);
    case 9
        vec = [ss(3,2); ss(1,3); ss(2,1)];
end
if isa(ss, 'sym')
    vec = simplify(vec);
end
end
