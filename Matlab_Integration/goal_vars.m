function goal_vars = goal_vars()
    x_goal = 10;                                            % goal position [m]
    y_goal = 10;                                            % goal position [m]
    psi_goal = deg2rad(270);                                % goal heading [rad]
    goal_vars = [x_goal,y_goal,psi_goal];                   % output matrix

end