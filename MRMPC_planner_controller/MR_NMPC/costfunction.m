function cost = costfunction(cc, np, ny, Ts, x0, model_type,costFunc,yd,t0)
%Function computing the cost function over np steps
% Author: Mohamed Elobaid DIAG Sapienza 2019
   u = cc;
    cost = 0;
    % check for defined cost function
   
    x = zeros(ny,np);
    y = zeros(ny,np);
    x(:,1) = x0;
    u0 = cc;
    cost_flag = costFunc();
    if cost_flag ~= 0
        %[x,y] =  OpenloopPrediction(np, Ts, x0,u, model_type,ny,t0);
        if (strcmp(model_type, 'DT'))
            for k = 1:np
                [x(:,k), y(:,k)] = Prediction_model(x0,u0,Ts,t0);
                x0 = x(:,k);
            end
        elseif (strcmp(model_type, 'CT'))
                error('Provide a DT model, or an odefun handle');
        end
        cost = cost+stage_cost(np,ny, y,yd, u0(:,k));
        cost = cost+terminal_cost(x0);
    elseif cost_flag == 0
        error('Provide a cost function');
    end
end