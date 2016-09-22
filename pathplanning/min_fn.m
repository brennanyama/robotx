function i_min = min_fn(OPEN, OPEN_COUNT, goal, G, RHS)
%Function returns the node with the minimum value.
    temp_array = [];
    k = 1;
    flag = 0;
    goal_index = 0;
    for j = 1 : OPEN_COUNT
        if(OPEN(j, 1) == 1)
            temp_array(k, :) = [OPEN(j, :), G(OPEN(j, 2), OPEN(j, 3)), RHS(OPEN(j, 2), OPEN(j, 3)), j];
            if (OPEN(j, 2) == goal(1) && OPEN(j, 3) == goal(2))
                flag = 1;
                goal_index = j;
            end
            k = k + 1;
        end
    end
    if flag == 1
        i_min = goal_index;
    end
    if (size(temp_array) ~= 0)
        [min_fn, temp_min] = min(temp_array(:, 7));
        i_min = temp_array(temp_min, 8);
    else
        i_min = -1;
    end
end
