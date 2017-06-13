function [disc_eqs]=discri(eqs,x,method)
%INPUT:
%   eqs: first order differential equations,of form f(t,y), to be discretized
%   x: state variables
%   method: string of 2 chars, representing discretizing scheme
%           'FE' for "Forward Euler"
%           'RK' for "Runge-Kutta 4"
%           others can be added easily
%OUTPUT:
%   disc_eqs: discretized verison of eqs

if method == 'FE' %Forward Euler method
    syms h
    disc_eqs = eqs * h + x;
elseif method == 'RK'  %Explicit Runge-Kutta (4) method
    syms u h
    f = matlabFunction(eqs, 'Vars', {u,[x(1);x(2);x(3);x(4)]});
    k1 = eqs;
    k2 = f(u,x+(h./2).*k1);
    k3 = f(u,x+(h./2).*k2);
    k4 = f(u,x + h.*k3);
    disc_eqs = x + (h./6).*(k1 + 2*k2 + 2*k3 + k4);
end
end