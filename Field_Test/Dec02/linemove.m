function [x, y] = linemove(currX, currY, axis)
%This function moves the robot in a line. For axis,
%1 = x-axis
%2 = y-axis
%3 = (x=y) diagonal
%4 = (y=-x) diagonal
    if(isequal(axis, 1))
        x = currX+10;
        y = currY;
    elseif (isequal(axis, 2))
        x = currX;
        y = currY+10;
    elseif (isequal(axis, 3))
        x = currX+10;
        y = currY+10;
    elseif (isequal(axis, 4))
        x = currX+10;
        y = currY-10;
    end
end