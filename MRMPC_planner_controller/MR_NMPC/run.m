%% This script runs the MR_Planner+NMPC controller algorithm
% Author Mohamed Elobaid, Sapienza 2019

clc
clear all

%% Simulation Parameters
Ts              = ;
r               = ;
delta           = ;
states          = ;
inputs          = ;
outputs         = ;

simTime         = ;
x0              = ;
u0              = ;
t0              = ;
model_type      = ; %specify DT or CT, current version only supports DT

%% NMPC and Planner parameters
np              = ;
nc              = ;
itr             = simTime/delta;
Q               = ; %optional
R               = ; %optional

%% Excuting the planner and NMPC

mpciter = 0;
while(mpciter < itr)
    % Check if t is not a multible of Ts
    if mod(t,Ts) == 0
        % Call the planner
        MR_Planner(@mr_model,delta);
    end
    [t,x,u] = NMPC(simTime, inputs,states, outputs, np, nc, Q, R, Ts, model_type);
    
    mpciter = mpciter + 1;
end

%% Plotting results