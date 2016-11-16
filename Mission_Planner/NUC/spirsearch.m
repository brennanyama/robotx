function [x, y] = spirsearch(R, turns, currX, currY);
    theta = [-1*pi*turns: 0.1 : pi*turns];
    r = 0:R/(length(theta)-1):R;
    x = r.*sin(theta);
    x = x+currX;
    y = r.*cos(theta);
    y = y+currY;
end