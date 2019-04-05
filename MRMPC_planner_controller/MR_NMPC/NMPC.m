%% This  general NMPC script based on Matlab's fmincon
% Author Mohamed Elobaid , Sapienza 2019
% Based on code by Lars Gruene, Juergen Pannek 2011


function [t,x,u] = NMPC(yd, nu,nx, ny, np, nc, Q, R, Ts, u0, x0, t0, model_type, options, ...
    l_constraints)

    t = [];
    x = [];
    u = [];
    t_Start = tic;
    [u_star, V_current, exitflag, output] = solveOptimalControlProblem ...
            (cost_function, np, nc, Ts, x0,u0, model_type, options,l_constraints);
    t_Elapsed = toc( t_Start );
    %   Store closed loop data
    t = [ t t0 ];
    x = [ x x0 ];
    u = [ u u_star];
end

function l = stage_cost(yd,y)
    n = length(yd);
    lk = 0;
    for i=1:n
       lk = lk+y(i)^2; 
    end
end

function lN = terminal_cost(x0)
 % TODO Define terminal cost (should i let the user define it ?)
end

function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (cost_function, np, nc, Ts, x0,u0, model_type, options,l_constraints)
    x = zeros(length(x0), np+1);
    x = computeOpenloopPrediction(np, nc, Ts, t0, x0,u0, model_type);
    
    % Set control and linear bounds
    A       = [];
    b       = [];
    Aeq     = [];
    beq     = [];
    lb      = [];
    ub      = [];
    
    for k=1:np
        [Anew,bnew,Aeqnew,beqnew,lbnew,ubnew] = l_constraints()
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end
    
    % Solve optimization problem
    [u, V, exitflag, output] = fmincon(@(u) costfunction(stage_cost, ...
        terminal_cost,u, np, nc, Ts, x0, model_type), u0, A, b, Aeq, beq, lb, ...
        ub , options);
end



function cost = costfunction(u, np, nc, Ts, x0, model_type, stage_cost, terminal_cost)
    cost = 0;
    % check for defined cost function
    cost_flag = costFunc();
    if cost_flag ~= 0
        x = zeros(length(x0), np+1);
        x = computeOpenloopSolution(np, nc, Ts, t0, x0,u, model_type);
        for k=1:np
            cost = cost+stage_cost(t0+k*Ts, x(:,k), u(:, k));
        end
        cost = cost+terminal_cost(t0+(np+1)*T, x(:,np+1));
    end
end


function x = computeOpenloopPrediction(np, nc, Ts, t0, x0,u0, model_type)
    % Should I initialize x ?
    x = zeros(length(x0),np+1);
    if (strcomp(model_type, 'DT'))
        x(:,1) = x0;
        for k = 1:np
            x(:,k+1) = Prediction_model(x0,u0,Ts);
            x0 = x(:,k+1);
        end
    elseif (strcomp(model_type, 'CT'))
        error('Provide a DT model, or an odefun handle');
    end
end

function [c, ceq] = nlconstraints(t,x,u)
% ToDo add dependence on x at time t


end


