function [destX, destY] = mission_plan(event_map, currX, currY)
%This function searches an event map and sets the next waypoint for the
%robot.
    % Buoy Color Constants
    RED = 1;
    GREEN = 2;
    
    % Spiral Search Pattern Parameters
    R = 5;          % Radius of sprial search pattern
    turns = 5;      % Number of turns for search
    
    if(max(event_map) == 0) % there are no events on the map
        x = [-1*pi*turns: 0.1 : pi*turns];
        r = [0:R/(length(x)-1):R];
        destX = r.*sin(x)+currX;
        destY = r.*cos(x)+currY;
    else
        red_loc = find(event_map == RED);      % Location of all known red buoys
        green_loc = find(event_map == GREEN);   % Location of all known green buoys
        
        
end