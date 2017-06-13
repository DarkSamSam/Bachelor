function eqs=dyn(x,dx,forces,levers, para)
%INPUT:
%   x: state variables
%   dx: variables represtenting first derivatives of state variable
%   forces: all forces of all systems
%   levers: all lever moments of all systems
%   para: physical parameter of system
%OUTPUT:
%   eqs: nonlinear state equations of the problem

%% system 1 - wheels
acc1 = [dx(2);0;0]; %accélération vector

eq1 = sum(forces(:,:,1),2) == para.M*acc1; %2nd Newton:F=ma

%% system 2 -pendulum

% Cartesian to polar coordinates
e_rho = [sin(x(3)),cos(x(3)),0]; %vector basis for polar coordinates
e_theta = [cos(x(3)),-sin(x(3)),0]; %vector basis for polar coordinates

%add general formula 4 acc in polar coords

acc2 = [para.L*x(4)^2;para.L*dx(4);0]; %polar acceleration vector

eq2 = [dot(sum(forces(:,:,2),2),e_rho);dot(sum(forces(:,:,2),2),e_theta);0] == para.m*acc2; %2nd Newton:F=ma in polar coords

%conservation of angular momentum
kin = [0;0;para.I*dx(4)]; %angular momentum

eq3 = sum(levers(:,:,2)) == kin; % Conservation of angular momentum

%% isolating x_dotdot and theta_dotdot
syms N P positive
eqns = [eq1;eq2;eq3];   %system of equations, some of which are redundant
[s1,s2] = solve(eq2(1),eq3(3),N,P); %I don't kwon why, but I can't solve the system directly, I have to do this intermediate solve
N = s1;
P = s2;
eqns = subs(eqns);
[x2_dot,x4_dot] = solve(eqns(1),eqns(5),dx(2),dx(4));

x2_dot = subs(x2_dot);
x4_dot = subs(x4_dot);

%% state equations:
eqs(1,1) = x(2);
eqs(2,1) = x2_dot;
eqs(3,1) = x(4);
eqs(4,1) = x4_dot;
end