%% This script runs the MR_Planner+NMPC controller algorithm
% Author Mohamed Elobaid, Sapienza 2019

clc
clear all

%% Simulation Parameters
Ts              = 1;
r               = 2;
delta           = 0.5;
nx              = 3;
nu              = 1;
ny              = 1;

simTime         = 20;
x_init          = [0;0;0];
u_init          = 0.0;
t0              = 0.0;
model_type      = 'DT'; %specify DT or CT, current version only supports DT

%% NMPC and Planner parameters
np              = 2;
nc              = np;            %for now we will assume this TODO add nc<np
itr             = simTime/delta;
Q               = eye(3); %optional
R               = 0; %optional

% To define constraints, user must fill the l_constraints function at end
% of this script

options         = optimoptions('fmincon','Display','iter','Algorithm','sqp'); %opt options for solver

%% Excuting the planner and NMPC

u0 = zeros(length(u_init),nc);    % question, should i consider u0 = u_init,
                                  % over nc, or just zeros over n_c
for k = 1:nc
    u0(:,k) = u_init;
end

t_measure = t_init;
x_measure = x_init;               % initialization

% Checking for prediction model in path/directory
pred_flag = predModel();
    
if pred_flag == 0                                
    v = MR_Planner(@mr_model,delta); % a matrix of size 2np*ny 
    mpciter = 0;
    while(mpciter < itr)
        %   Obtain new initial value
        [t0, x0] = measuredValue( tmeasure, xmeasure );
        % Check if t is not a multible of Ts
        if mod(t,Ts) == 0
            % Call the planner
            v = MR_Planner(@mr_model,delta);
        end
        yd = ref_shift(v, mpcitr);
        %   Solve the optimal control problem
        [t,x,u_star] = NMPC(yd, nu,nx, ny, np, nc, Q, R, Ts, u0, x0, t0, model_type, options, ...
            l_constraints);
        %   Apply control to process
        [tmeasure, xmeasure] = applyControl(Prediction_model, Ts, t, x, u_star(:,1));  % apply u0
        u0 = u_star; 
        mpciter = mpciter + 1;
    end
end


%% Plotting results
% TODO

%% Checking for necessary user defined models and costs

% Check for prediction model

function pred_flag = predModel()
if exist('Prediction_model') > 0
     disp('STATUS: Prediction model obtained');
     pred_flag = 0;
else
     error('No prediction model in current path/directory');
end
end

%% Applying control to process

function [t0, x0] = measuredValue( tmeasure, xmeasure )
        t0 = tmeasure;
        x0 = xmeasure;
end


function [t, x] = applyControl(Prediction_model, Ts, t0, x0, u)
        u0 = u(:,1);
        x = Prediction_model(x0,u0,Ts);
        t = t0+Ts;
end

function  [A,b,Aeq,beq,lb,ub] = l_constraints()
        A   = [];
        b   = [];
        Aeq = [];
        beq = [];
        lb  = lb;
        ub  =  ub;
end





