function [A,B,C,D]=lini(dx,x,u)
%INPUT:
%   dx: state equation, discrete or continous
%   x: state variables
%   u: input force variable
%OUTPUT:
%   A,B,C,D: matrixes for linear state equation

A = jacobian(dx, x); %jacobian of state equations w.r.t. state variables
B = jacobian(dx, u); %jacobian of state equations w.r.t input

x1=0;x2=0;x3=0;x4=0;u=0; %nominal trajectories

A = double(subs(A)); %evaulation of jacobian around nominal trajectories
B = double(subs(B)); %evaluation of jacobian around nominal trajectories
C = [1,0,0,0;0,0,1,0]; %outputs of interest
D = [0;0]; %input does not influence directly output
end