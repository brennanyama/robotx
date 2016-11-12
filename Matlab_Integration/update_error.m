function error = update_error(k,x,goal_vars,control_maximums,error)

    % Update errors
    error(1,k) = goal_vars(1)-x(1,k);               % x position goal error
    error(2,k) = goal_vars(2)-x(2,k);               % y position goal error
    error(3,k) = goal_vars(3)-x(3,k);               % psi heading goal error
    error(4,k) = sqrt((goal_vars(1)-x(1,k))^2+...   % path distance error
        (goal_vars(2)-x(2,k))^2);
    error(5,k) = atan2(goal_vars(2)-x(2,k),...      % path heading error
        goal_vars(1)-x(1,k))-x(3,k);
    error(6,k) = control_maximums(1)-x(4,k);        % surge velocity max speed error
    error(7,k) = control_maximums(2)-x(5,k);        % sway velocity max speed error
    error(8,k) = 0-x(4,k);                          % surge velocity stop error
    error(9,k) = 0-x(5,k);                          % sway velocity stop error
    
end