%% Balancing robot, main script
clear all;
close all;
clc;

%% Define states
syms x1 x2 x3 x4 f1xu f2xu f3xu f4xu u positive
state = [x1;x2;x3;x4];% x,dx,theta,dtheta
fxu = [f1xu;f2xu;f3xu;f4xu]; %state equation

%% Get Parameters
para.I = 0.06; %inertia of pendulum [kg*m^2]
para.M = 0.226; %mass of wheels [kg]
para.m = 0.3; %mass of pendulum [kg]
para.g = 9.81; % gravity [m/s^2]
para.L = 0.03; %distance from pivot point to center of mass of pendulum [m]
para.h = 1./1200; %sampling time
%vers arbitrary approximations for now

%motor parameters:
paraMot.Km = 0.01; % motor torque constant [Nm/A] or [V/(rad s)]
paraMot.R = 1; % internal resistance [Ohm]
paraMot.r = 0.025; % radius of the wheels [m]

%% Forces and levers
%note to self: maybe put all this in external script/function ?
syms N P positive %internal cohesion forces
Mg = [0;para.M*para.g;0]; %gravific force
T = [N;P;0]; %internal force
F_motor = motor_force(u,state,paraMot); 


mg = [0;para.m*para.g;0]; %gravific force
Fin = [-para.m*f2xu;0;0]; %inertia "force" on pendulum from cart acceleration 

l1 = cross(-[sin(state(3));cos(state(3));0]*para.L,-T); %lever exerted by cart on pendulum

forces = cat(3,[Mg,T,F_motor],[-T,mg,Fin]); %all forces of system on 1 line, every system is new line
levers = cat(3,zeros(3,1),l1); %all levers of system on 1 line, every system is new line

%% Non linear equations
fxu = dyn(state,fxu,forces,levers,para); %non linear state equations

%% Discretization
disc_eqs = discri(fxu,state,'RK');
syms h
h = para.h;
disc_eqs = subs(disc_eqs);

%% Linearization
% [A,B,C,D] = lini(fxu,state,u);   %returns linear matrices, usung first order Taylor approximation
% fxu_l = A*state + B*u; %linear state equations
[phi,gamma,C,D] = lini(disc_eqs,state,u);
disc_eqs_l = phi*state + gamma*u;

%% Stablility analysis

%% Open Loop
   tspan = [0 20]; %simulation interval, this will need to change a little, after discretization is implemented
  y0 = [0 0 0.5 0];   %initial conditions on states
  [open_times,open_states,open_feedback] = open_control(disc_eqs,state, 10, y0, para.h); %last input should be filename if graph is to be saved


%% PID
% [pid_times,pid_states,pid_feedback] = pid_control(disc_eqs,state,0,y0,60,1,20,10,para.h);


%% LQR
 Q = C'*C;
 Q(1,1) = 1; % x position weight in LQR
 Q(3,3) = 500;   % theta angle weight in LQR
 R = 1;  %weight of input in LQR
[lqr_times,lqr_states,lqr_feedback] = lqr_control(phi,gamma,Q,R,y0,10,para.h);

%% Graphs
%note: figure ids 40-49 are reserved for open loop figures, 50-59 for pid
%contolled figures, and 60-69 for lqr controlled figures
% figure(40);
% displaysimu(open_times,open_states,open_feedback,''); %last input should be a filename if a gif is wanted
% figure(50);
% displaysimu(pid_times,pid_states,pid_feedback,'');
% figure(51);
% displaysimu(pid_l_times,pid_l_states,pid_l_feedback,'');
% figure(60);
% displaysimu(lqr_times,lqr_states,lqr_feedback,'');

%% Animation
% figure(41);
% anim(open_times,open_states,para,'');   %last input should be a filename if a gif is wanted
% figure(52);
% anim(pid_times,pid_states,para,'');
% figure(53);
% anim(pid_l_times,pid_l_states,para,'');
% figure(61);
% anim(lqr_times,lqr_states,para,'');

