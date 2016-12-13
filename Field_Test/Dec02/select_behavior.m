function behavior = select_behavior(k,x,control_tolerances,error)

%% Waypoint Navigation Code

    if (error(4,k) >= control_tolerances(1))                % robot is outside the approach radius
        if (error(5,k) >= control_tolerances(3))            % robot is outside path heading tolerance
            behavior = 1;                                  % control psi to path heading and control u to medium fast
        else                                                % robot is inside path heading tolerance
            behavior = 2;                                  % control psi to path heading and control u to fast
        end
    else                                                    % robot is inside approach radius
        if (error(4,k) >= control_tolerances(2))            % robot is outside in the goal radius
            if (error(5,k) >= control_tolerances(3))        % robot is outside path heading tolerance
                behavior = 3;                              % control psi to path heading and control u to slow
            else                                            % robot is inside path heading tolerance
                behavior = 4;                              % control psi to path heading and control u to medium slow
            end
        else                                                % robot is inside goal radius
            if (x(4,k) >= 0.25)                             % robot is moving
                behavior = 5;                              % control u to 0 m/s
            else                                            % robot is largely stopped
                if (error(3,k) >= control_tolerances(4))    % robot is not facing goal heading
                    behavior = 6;                          % control psi to goal heading and control u to 0 m/s
                else                                        % robot is facing goal heading
                    behavior = 7;                          % station keep
                end
            end
        end
    end
end
