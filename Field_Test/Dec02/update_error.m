function error = update_error(k,x,goal_vars)
    temp = zeros(10, 1);
    % Update errors
    temp(1,1) = goal_vars(1)-x(1,k);                   % x position goal error
    temp(2,1) = goal_vars(2)-x(2,k);                   % y position goal error
    temp(3,1) = goal_vars(3)-x(3,k);                   % psi heading goal error
    temp(4,1) = sqrt((goal_vars(1)-x(1,k))^2+...       % path distance error
        (goal_vars(2)-x(2,k))^2);   
    temp(5,1) = atan2(sin(atan2(goal_vars(2)-x(2,k),...    % HO FUCK EQUATION (there's a good story behind this one)
        goal_vars(1)-x(1,k))-x(3,k)),...
        cos(atan2(goal_vars(2)-x(2,k),...
        goal_vars(1)-x(1,k))-x(3,k)));
    temp(6,1) = 0-x(4,k);                              % surge velocity stop error   
    temp(7,1) = 0.10-x(4,k);                           % surge velocity slow error
    temp(8,1) = 0.25-x(4,k);                           % surge velocity medium slow m/s error
    temp(9,1) = 0.5-x(4,k);                            % surge velocity medium fast m/s error
    temp(10,1) = 2.00-x(4,k);                          % surge velocity fast m/s error
    error = temp;
end
