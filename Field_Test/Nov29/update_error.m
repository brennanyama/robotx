function error = update_error(k,x,meas,goal_vars,error)

    % Update errors
    error(1,k) = goal_vars(1)-x(1,k);                   % x position goal error
    error(2,k) = goal_vars(2)-x(2,k);                   % y position goal error
    error(3,k) = goal_vars(3)-x(3,k);                   % psi heading goal error
    error(4,k) = sqrt((goal_vars(1)-x(1,k))^2+...       % path distance error
        (goal_vars(2)-x(2,k))^2);   
    error(5,k) = atan2(sin(atan2(goal_vars(2)-x(2,k),...    % HO FUCK EQUATION (there's a good story behind this one)
        goal_vars(1)-x(1,k))-x(3,k)),...
        cos(atan2(goal_vars(2)-x(2,k),...
        goal_vars(1)-x(1,k))-x(3,k)));
    error(6,k) = 0-x(4,k);                              % surge velocity stop error   
    error(7,k) = 0.10-x(4,k);                           % surge velocity slow error
    error(8,k) = 0.25-x(4,k);                           % surge velocity medium slow m/s error
    error(9,k) = 0.5-x(4,k);                            % surge velocity medium fast m/s error
    error(10,k) = 2.00-x(4,k);                          % surge velocity fast m/s error
    
end
