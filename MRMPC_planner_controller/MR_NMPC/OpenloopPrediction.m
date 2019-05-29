function [x,y] = OpenloopPrediction(np, Ts, x0,u0, model_type,ny,t0)
    % Should I initialize x ?
    x = zeros(length(x0),np);
    y = zeros(ny,np);
    if (strcmp(model_type, 'DT'))
        for k = 1:np
            [x(:,k), y(:,k)] = Prediction_model(x0,u0,Ts,t0);
            x0 = x(:,k);
        end
    elseif (strcmp(model_type, 'CT'))
        error('Provide a DT model, or an odefun handle');
    end
end

