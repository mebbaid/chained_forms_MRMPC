function cost = costfunction(u0, np, ny, Ts, x0, model_type,costFunc,yd,t0)
%Function computing the cost function over np steps
% Author: Mohamed Elobaid DIAG Sapienza 2019

    cost = 0;
    % check for defined cost function
    cost_flag = costFunc();
    if cost_flag ~= 0
        x = zeros(length(x0), np);
        y = zeros(ny,np);
        [x,y] =  OpenloopPrediction(np, Ts, x0,u0, model_type,ny,t0);
        for k=1:np
            cost = cost+stage_cost(ny, y(:,k),yd(:,k));
        end
        cost = cost+terminal_cost(x0);
    elseif cost_flag == 0
        error('Provide a cost function');
    end
end