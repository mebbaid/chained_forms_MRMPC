% init for testing

clc 
clear all

%options = optimoptions('fmincon','Display','iter','Algorithm','interior-point','OptimalityTolerance', 1e-6);

Ts = 1;
t0 = 0.0; %initial time

np = 20;
nc = np;


model = 'DT';
ny = 3;
nu = 2;

yd = ones(ny,np);
x0 = [1;3;-1];
u0 = zeros(2,np); % initial guess

%[ustar,vstar] = mrmpc(np, ny, Ts, x0,u0, model, options,costFunc,yd,t0);

Obj_index = @(cc) costfunction(cc, np, ny, Ts, x0, model,costFunc,yd,t0);
%Obj_index = @(x)1+x(1)/(1+x(2)) - 3*x(1)*x(2) + x(2)*(1+x(1));
[u, Vstar, exitflag, output] = fmincon(Obj_index, u0, [], [], [], [], [], ...
        [] , [])%,  options)

function cost_flag = costFunc()
   cost_flag = 0;
    if exist('stage_cost') > 0
         disp('STATUS: stage cost obtained');
         cost_flag = 1;
    else
         error('No stage cost in current path/directory');
    end
end