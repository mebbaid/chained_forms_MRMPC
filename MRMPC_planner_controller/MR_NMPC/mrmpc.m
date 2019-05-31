function [ustar,vstar] = mrmpc(np, ny, Ts, x0,u0, model, options,costFunc,yd,t0)
%mrmpc solves the optimal control problem in a recding horizon fashion
% Author: Mohamed Elobaid DIAG Sapienza 2019

[ustar, vstar, exitflag, output] = solveOCP( np, ny, Ts, x0,u0, model, options,costFunc,yd,t0);

if exitflag < 0
   error('problem isnt feasible, try different opt alg.') 
end

end

