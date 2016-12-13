function adj = limit_waypoint(waypoint)
%This function ensures that the waypoints passed to the motion controller
%are within the map world limits.
    min = -51.25;
    max = 51.15;
    x = waypoint(1);
    y = waypoint(2);
    
    if lt(x, min)
        x = min+0.01;
    end
    if gt(x, max)
        x = max-0.01;
    end
    if lt(y, min)
        y = min+0.01;
    end
    if gt(y, max)
        y = max-0.01;
    end
    
    adj = [x, y];
end