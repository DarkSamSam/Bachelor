function [t,y,output]=open_control(eqs, x, tspan, y0, h)
%INPUT:
%   eqs: discrete system equations, either linear or non linear
%   x: state variables
%   tspan: length of time so simulate [s]
%   y0: initial sates
%   h: time step
%OUTPUTS:
%   t: vector of sampling times
%   y: vector of solutions the state equation at sampling times of t
%   output: vector of controller inputs at sampling time t, which here is a
%   null vecotor, since there is no controller

syms u
eqs = subs(eqs);
f = matlabFunction(eqs, 'Vars', {u,[x(1);x(2);x(3);x(4)]}); %transforms symbolic equations into parametric equations

n = floor(tspan/h);
y = zeros(n,4);
t = zeros(1,n);
%intital setup
y(1,1) = y0(1);
y(1,2) = y0(2);
y(1,3) = y0(3);
y(1,4) = y0(4);
output = zeros(1,n);
for i=2:n
    xx = f(output(i), y(i-1,:)');
    y(i,1) = xx(1);
    y(i,2) = xx(2);
    y(i,3) = xx(3);
    y(i,4) = xx(4);
    t(i) = i*h; %timestep incrementation
end
end