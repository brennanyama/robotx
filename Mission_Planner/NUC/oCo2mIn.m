function [xIn, yIn] = oCo2mIn(xOg, yOg)
%Converts occupancy grid coordinates to MATLAB matrix indicies.
    xIn = round(10*(xOg+50), -1);
    yIn = round(10*(yOg+50), -1);
end