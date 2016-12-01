function [x, y] = spirsearch(R, turns, currX, currY);
    addX = 5;               % Distance in meters in front of the robot to center search aroud
    addY = 0;               
    theta = [-1*pi*turns: 0.1 : pi*turns];
    r = R:-(R-0.5)/(length(theta)-1):0.5;
    x = r.*cos(theta);
    x = x+currX+addX;
    y = r.*sin(theta);
    y = y+currY+addY;
end
