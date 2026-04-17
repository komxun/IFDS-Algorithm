function length = norm_ubar(rho0, sigma0, Gamma, n, t, u, d, d0)

%     c1 = u'*(n)*n'*u;
%     c2 = u'*(t)*n'*u;
%     c3 = u'*(n)*n'*t*n'*u;
%     c4 = u'*(n)*t'*u;
%     c5 = u'*(n)*t'*(n)*n'*u;
%     c6 = u'*(n)*(t)'*t*n'*u;
%     c7 = n'*n;
%     
%     nn = norm(n);
%     tt = norm(t);
%     
    rho = rho0 * exp(1-1/(d0*d));
    sigma = sigma0 * exp(1-1/(d0*d));
%     
%     s1 = -2*c1/(abs(Gamma)^(1/rho)*c7);
%     s2 = (c2+c4)/(abs(Gamma)^(1/sigma)*tt*nn);
%     s3 = c1/(abs(Gamma)^(2/rho)*c7);
%     s4 = -c3/(abs(Gamma)^(1/rho)*abs(Gamma)^(1/sigma)*tt^2*nn^2);
%     s5 = -c5/(abs(Gamma)^(1/rho)*abs(Gamma)^(1/sigma)*tt*nn*c7);
%     s6 = c6/(abs(Gamma)^(2/sigma)*tt^2*nn^2);
% 
%     length = s1 + s2 + s3 + s4 + s5 + s6;

    M = eye(3) - n*n'/(abs(Gamma)^(1/rho)*(n')*n)...
                + t*n'/(abs(Gamma)^(1/sigma)*norm(t)*norm(n));
    length = (M*u)'*(M*u);
%     length = sqrt(length)/(d^2);

end