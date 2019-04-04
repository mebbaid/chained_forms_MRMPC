%% This  general NMPC script based on Matlab's fmincon
% Author Mohamed Elobaid , Sapienza 2019
% Based on code by Lars Gruene, Juergen Pannek 2011


function [x,u, y] = NMPC(simTime, inputs,states, outputs, np, nc, Q, R, Ts, model_type)

    u0 = zeros(inputs, nc);
    t = [];
    x = [];
    u = [];
    n = states;
    % Check for prediction model in path
    pred_flag = predModel();
    itr = simTime/Ts ;  
    if pred_flag ~= 0 
            %   Obtain new initial value
            [t0, x0] = measureInitialValue ( tmeasure, xmeasure );
            %   Solve the optimal control problem
            t_Start = tic;
            [u_new, V_current, exitflag, output] = solveOptimalControlProblem ...
                (stage_cost, terminal_cost,u, np, nc, Ts, x0,u0, model_type);
            t_Elapsed = toc( t_Start );
            %   Store closed loop data
            t = [ t tmeasure ];
            x = [ x xmeasure ];
            u = [ u u_new(:,1)];
            %   Prepare restart
            u0 = shiftHorizon(u_new);
            %   Apply control to process
            [tmeasure, xmeasure,ymeasure] = applyControl(system, T, t0, x0, u_new, ...
                atol_ode_real, rtol_ode_real, type);
            mpciter = mpciter+1;
    end
 end
    


% Check stage cost function

function cost_flag = costFunc()
if exists('stage_cost.m') > 0 
     disp('STATUS: stage cost obtained');
     cost_flag = 0;
else
     error('No stage cost in current path/directory, using default cost');
     cost_flag = 1;
end
end

% Check for prediction model

function pred_flag = predModel()
if exist('Prediction_model') > 0
     disp('STATUS: Prediction model obtained');
     pred_flag = 0;
else
     error('No prediction model in current path/directory');
     pred_flag = 1;
end
end

% update states measurements

function [t0, x0] = measureInitialValue ( tmeasure, xmeasure )
    t0 = tmeasure;
    x0 = xmeasure;
end



function [tapplied, xapplied] = applyControl(system, T, t0, x0, u, ...
                                atol_ode_real, rtol_ode_real, type)
    xapplied = dynamic(system, T, t0, x0, u(:,1), ...
                       atol_ode_real, rtol_ode_real, type);
    tapplied = t0+T;
end



function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end



function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (stage_cost, terminal_cost,u, np, nc, Ts, x0,u0, model_type)
    x = zeros(N+1, length(x0));
    x = computeOpenloopPrediction(np, nc, Ts, t0, x0,u0, model_type);
    
    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    for k=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+k*T,x(k,:),u0(:,k));
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



function cost = costfunction(stage_cost, terminal_cost,u, np, nc, Ts, x0, model_type)
    cost = 0;
    % check for defined cost function
    cost_flag = costFunc();
    if cost_flag ~= 0
        x = zeros(length(x0), np+1);
        x = computeOpenloopSolution(np, nc, Ts, t0, x0,u, model_type);
        for k=1:np
            cost = cost+stage_cost(t0+k*Ts, x(:,k), u(:, k));
        end
        cost = cost+terminalcosts(t0+(np+1)*T, x(:,np+1));
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
