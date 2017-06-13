function [t,y,output]=simu(eqs, x, input, tspan, yo, filename)
%INPUT:
%   eqs: first order differential equation, of form f(t,y)
%   x: state variables
%   input: input force as symbolic function of t
%   tspan: interval of simulation of form [t0 tfin]
%   y0: intitial conditions of forme [x1_0 x2_0 x3_0 x4_0]
%   filename: filename to save graph in form of string. use '' if save is
%   not wanted
%OUTPUT:
%   t: timesteps where the numerical solution was computed
%   y: numerical solutions of diff equations at timestep of t

syms t positive
u = input;  %input must be a symbolic function of t (later also of state, when feedback)
eqs = subs(eqs);
handle = matlabFunction(eqs, 'Vars', {t,[x(1);x(2);x(3);x(4)]});
[t,y] = ode45(handle,tspan,yo); %solver based on RK4
disturbance = matlabFunction(input, 'Vars', 't');
output = zeros(1,length(t));
for i = 1:length(t)
    output(i) = disturbance(t(i));
end
t = t';
% displaysimu(t,y,output);   %plots simulation
% note to self: make graphical part completely independ of numerical part ?
% if filename ~= ''
%     print(filename,'-dpng');    %save graph to file
% end
end