function behavior = select_behavior(k,x,control_tolerances,error,behavior)
    
    if error(4,k) > control_tolerances(1)                                   % robot is not within goal radius tolerance
        if abs(error(5,k)) > control_tolerances(2)                          % robot is not on within path heading tolerance
            behavior(1,k) = 1;
        else
            behavior(1,k) = 2;
        end
    else                                                                    % robot is within goal radius tolerance
        if abs(x(4,k)) || abs(x(5,k)) > control_tolerances(4)               % drive velocity to zero
            behavior(1,k) = 3;
        else
            behavior(1,k) = 4;
        end
    end

end