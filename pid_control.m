function [t,y,output]=pid_control(eqs,x,setpoint,y0,kp,ki,kd,time,h)
%INPUT:
%   eqs: discrete system equations, either non linear or linear
%   x: state variables
%   setpoint: desired value of interest
%   y0: initial values
%   kp: proportional controller gain
%   ki: integral controller gain
%   kd: derivative controller gain
%   time: length of time to run simulation [s]
%   h: discrete time step
%OUTPUT:
%   t: vector of sampling times
%   y: vector of solutions the state equation at sampling times of t
%   output: vector of controller inputs at sampling time t

syms u
f = matlabFunction(eqs, 'Vars', {u,[x(1);x(2);x(3);x(4)]}); %transforms symbolic equations into parametric equations

n=floor(time/h);    %number of samples
y = zeros(n,4);
t = zeros(1,n);

%intital setup
y(1,1) = y0(1);
y(1,2) = y0(2);
y(1,3) = y0(3);
y(1,4) = y0(4);
lasterr=setpoint-y(1,3);  %keeps track of the last error
integral=0; %keeps track of total error over time
output = zeros(1,n);

% simulation with pid controller. Here only the tilt angle is controlled
for i=2:n
    measured = y(i-1,3);    %last calculated angle value
    err = setpoint - measured;  %error
    integral = integral + err * h;  %adds error to total error
    derivative = (err - lasterr) / h;   %discrete numerical approximation of derivative at current timestep (Euler Method)
    output(i) = kp * err + ki * integral + kd * derivative;    %pid output, which will be used to feedback into equations
    lasterr = err;  %setting previous error as current error for next iteration
    xx = f(output(i), y(i-1,:)');  %computation of states at current time step, with pid input
    %adding current input to solution vector
    y(i,1) = xx(1);
    y(i,2) = xx(2);
    y(i,3) = xx(3);
    y(i,4) = xx(4);
    t(i) = i*h; %timestep incrementation
end
end
