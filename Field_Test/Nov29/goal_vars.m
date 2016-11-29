function goal_vars = goal_vars()
    x_goal = 0;                                            % goal position [m]
    y_goal = 0;                                            % goal position [m]
    psi_goal = deg2rad(0);                                % goal heading [rad]
    goal_vars = [x_goal,y_goal,psi_goal];                   % output matrix

end