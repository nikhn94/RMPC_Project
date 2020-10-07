%% ME47035 Robot Motion Car MPC Path Planner Project
clc;
clear;
%% Define the car initial conditions
Car = Car_MPC_intial_conditions;
%% Car configuration
Car.Xpos = Car.Speed