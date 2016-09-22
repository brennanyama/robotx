function exp_array = expand_array(xNode, yNode, path_cost, goal, CLOSED, dimm)
    exp_array = [];
    exp_count = 1;
    c2 = size(CLOSED, 1);
    
    for k = 1: -1: -1
        for j = 1: -1: -1
            if (k ~= j || k ~= 0)
                Sx = xNode + k;
                Sy = yNode + j;
                
                if( (Sx > 0 && Sx <= dimm(1)) && (Sy > 0 && Sy <= dimm(2)) )
                    flag = 1;
                    
                    for c1 = 1:c2
                        if (Sx == CLOSED(c1, 1) && Sy == CLOSED(c1, 2))
                            flag = 0;
                        end
                    end
                    
                    if (flag == 1)
                        exp_array(exp_count, 1) = Sx;
                        exp_array(exp_count, 2) = Sy;
                        exp_array(exp_count, 3) = path_cost + distance([xNode, yNode], [Sx, Sy]);
                        exp_array(exp_count, 4) = distance(goal, [Sx, Sy]);
                        exp_array(exp_count, 5) = exp_array(exp_count, 3) + exp_array(exp_count, 4);
                        exp_count = exp_count + 1;
                    end
                end
            end
        end
    end
end
