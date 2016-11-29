function error_col = update_error(k,x,meas,goal_vars,control_maximums)
    temp = zeros(9, 1);
    % Update errors
    temp(1,1) = goal_vars(1)-x(1,k);                   % x position goal error
    temp(2,1) = goal_vars(2)-x(2,k);                   % y position goal error
    temp(3,1) = goal_vars(3)-x(3,k);                   % psi heading goal error
    temp(4,1) = sqrt((goal_vars(1)-x(1,k))^2+...       % path distance error
        (goal_vars(2)-x(2,k))^2);   
    temp(5,1) = atan2(goal_vars(2)-x(2,k),...          % path heading error
         goal_vars(1)-x(1,k))-x(3,k);
%     error(5,k) = atan2(goal_vars(2)-x(2,k),...          % path heading error
%         goal_vars(1)-x(1,k));
    temp(6,1) = control_maximums(1,1)-meas(4,k);         % surge velocity max speed error
    temp(7,1) = control_maximums(1,2)-meas(5,k);         % sway velocity max speed error
    temp(8,1) = 0-meas(4,k);                           % surge velocity stop error
    temp(9,1) = 0-meas(5,k);                           % sway velocity stop error    
    error_col = temp;
end