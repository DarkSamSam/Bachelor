function [ Force ] = motor_force( Voltage, State, paraMot )
%MOTOR_FORCE given the parameters of the motor and the input voltage this
%function gives the resulting motor force based on a simple model of a DC
%machine
%   The inductive behavior of La is assumed to be very low as we will not
%   have too big steps in its voltage ??

Km = paraMot.Km; % motor torque constant [Nm/A] or [V/(rad s)]
r  = paraMot.r; % radius of the wheels [m]
R  = paraMot.R; % internal resistance [Ohm]

Force = [Voltage*Km/(R*r) - State(2)*Km^2/(R*r^2);0;0]; %only in direction of x

end

