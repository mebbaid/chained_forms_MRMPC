function [ustar, Vstar, exitflag, output] = solveOCP ...
    (np, ny, Ts, x0,u0, model_type, options,costFunc,yd,t0)
    x = zeros(length(x0), np);
    x = OpenloopPrediction(np, Ts, x0,u0, model_type,ny,t0);
    
    % Set control and linear bounds
    A       = [];
    b       = [];
    Aeq     = [];
    beq     = [];
    lb      = [];
    ub      = [];

%----------- incorporating user constraints
%     for k=1:np
%         [Anew,bnew,Aeqnew,beqnew,lbnew,ubnew] = l_constraints()
%         A = blkdiag(A,Anew);
%         b = [b, bnew];
%         Aeq = blkdiag(Aeq,Aeqnew);
%         beq = [beq, beqnew];
%         lb = [lb, lbnew];
%         ub = [ub, ubnew];
%     end


 %---------- Solve optimization problem
 
[ustar, Vstar, exitflag, output] = fmincon(@(u) costfunction(u, np, ny, Ts, x0, model_type,costFunc,yd,t0), u0, A, b, Aeq, beq, lb, ...
        ub , [],  options);
end

