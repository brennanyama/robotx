function [m, x, y] = findmidpoint(x1, y1, x2, y2)
%This function finds the slope and midpoint given the X and Y coordinates
%of two points.
    x = 0.5*(x1+x2);
    y = 0.5*(y1+y2);
    m = (y2-y1)/(x2-x1);
end