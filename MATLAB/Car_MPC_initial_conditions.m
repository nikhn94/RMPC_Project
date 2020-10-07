%% ME47035 Robot Motion Car MPC Path Planner Project
clc;
clear;
function Car  =  Car_MPC_intial_conditions

%% Define the car initial conditions
Car.Length   = 1.5; % in m
Car.Speed    = 40;  % in km/hr
Car.XPos     = 0;   % in m
Car.YPos     = 0;   % in m
Car.Orient   = 0;   % in degree
Car.Steering = 0;   % in degree

end