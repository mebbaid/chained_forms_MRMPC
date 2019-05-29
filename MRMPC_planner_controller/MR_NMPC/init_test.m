% init for testing

clc 
clear all

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point','OptimalityTolerance', 1e-6);

Ts = 1;
t0 = 0.0; %initial time

np = 5;
nc = np;


model = 'DT';
ny = 1;
nu = 2;

yd = ones(1,np);
x0 = [0;0;0];
u0 = 0.7*ones(2,np); % initial guess



[ustar, Vstar, exitflag, output] = solveOCP( np, ny, Ts, x0,u0, model, options,costFunc,yd,t0)


function cost_flag = costFunc()
   cost_flag = 0;
    if exist('stage_cost') > 0
         disp('STATUS: stage cost obtained');
         cost_flag = 1;
    else
         error('No stage cost in current path/directory');
    end
end