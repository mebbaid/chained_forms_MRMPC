function lk = stage_cost(ny, y,yd)
    n = ny;
    lk = 0;
    for i=1:n
       lk = lk+(yd(i)-y(i))^2; 
    end
end
