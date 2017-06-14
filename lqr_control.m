function [t,y,u]=lqr_control(phi,gamma,Q1,Q2,y0,time,h)
%INPUT:
%   phi,gamma: discrete linear state equation matrices
%   Q1,Q2: weights of each state and input in feedback loop
%   y0: initial conditions
%   time: interval of time to compute solutions on
%   h: discret time step
%OUTPUT:
%   t: vector of sampling times
%   y: vector of solutions the state equation at sampling times of t
%   u: vector of controller inputs at sampling time t

%Riccati equation
phi = double(phi);  %symbolic to double conversion
gamma = double(gamma);  %symbolic to double conversion
% S = sym('S', [4,4]);
% eqs = S == phi'*(S-S*gamma*((Q2+gamma'*S*gamma)\(gamma'*S)))*phi+Q1
% Sinf = solve(eqs(1,1),eqs(1,2),eqs(1,3),eqs(1,4),eqs(2,1),eqs(2,2),eqs(2,3),eqs(2,4),eqs(3,1),eqs(3,2),eqs(3,3),eqs(3,4),eqs(4,1),eqs(4,2),eqs(4,3),eqs(4,4));

[Sinf,L,G] = dare(phi,gamma,Q1,Q2);  %solution of discrete algebaric Riccati equation (DARE)
Kinf = inv(Q2+gamma'*Sinf*gamma)*gamma'*Sinf*phi;   %controller gains computation

n=floor(time/h);    %number of samples
y = zeros(n,4);
t = zeros(1,n);

%intital setup
y(1,1) = y0(1);
y(1,2) = y0(2);
y(1,3) = y0(3);
y(1,4) = y0(4);
u = zeros(1,n);

%time simulation of the system with LQR controlling tilt and position
for i=2:n
    u(i) = -Kinf*y(i-1,:)'; %state feedback with controller gains
    x = phi*y(i-1,:)' + gamma*u(i); %iteration solution
    y(i,1) = x(1);
    y(i,2) = x(2);
    y(i,3) = x(3);
    y(i,4) = x(4);
    t(i) = i*h; %timestep incrementation
end
end
