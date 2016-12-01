function [destX, destY] = mission_plan(event_map, currX, currY)
%This function searches an event map and sets the next waypoint for the
%robot.
    % Buoy Color Constants
    RED = 1;
    GREEN = 2;
    AXIS = 1;
    
    % Spiral Search Pattern Parameters
    R = 4;          % Radius of sprial search pattern
    turns = 3;      % Number of turns for search
    
    if(max(event_map) == 0) % there are no events on the map
        disp('No buoys found. Beginning search for buoys.');
        [destX, destY] = spirsearch(R, turns, currX, currY);   % Begin searching in a spiral pattern centered at current location
    else
        [red_locX, red_locY] = find(event_map == RED);      % Location of all known red buoys returned as column vectors
        [green_locX, green_locY] = find(event_map == GREEN);   % Location of all known green buoys returned as column vectors
        
        red_num = size(red_locX);           % red_locX and red_locY have the same dimensions
        red_num = red_num(1);
        
        green_num = size(green_locX);       % green_locX and green_locY have the same dimensions
        green_num = green_num(1);
        
        disp('Known Red Buoys: '); disp(red_num);
        disp('Known Green Buoys: '); disp(green_num);
        destX = red_locX(1); destY = red_locY(1);
        
        if (xor((red_num == 1), (green_num == 1)))
            if (red_num == 1)
                [destX, destY] = spirsearch(R, turns, red_locX(1), red_locY(1));       % Spiral search around the known red buoy
            elseif (green_num == 1)
                [destX, destY] = spirsearch(R, turns, green_locX(1), green_locY(1));   % Spiral search around the known green buoy
            end
        elseif(and((red_num == 1), (green_num == 1)))
            [m, x, y] = findmidpoint(red_locX(1), red_locY(1), green_locX(1), green_locY(1));       % Go to the midpoint of the Red and Green buoy
            [destX, destY] = linewaypoint(x, y, -1/m, 1, 30);                           % Waypoints along the line perpendicular to the line connecting the red and green buoy passing through the midpoint of the red and green buoy
        elseif (and((red_num == 2), (green_num == 2)))
            [m, destX, destY] = findmidpoint(red_locX(2), red_locY(2), green_locX(2), green_locY(2));    % Destination is the midpoint of the second pair of red and green buoys found.
        end
            
    end
end
