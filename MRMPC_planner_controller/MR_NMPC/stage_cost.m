function lk = stage_cost(np,ny, y,yd,u)
    n = ny;
    lk = 0;
    for j = 1:np
        for i=1:n
           lk = lk+(yd(i,j)-y(i,j))^2; 
        end
    end
end
