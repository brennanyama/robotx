function [xOg, yOg] = mIn2oCo(xIn, yIn)
%This function converts from MATLAB matrix indicies to occupancy grid
%coordinates.
    xOg = round(xIn/10-50,1);
    yOg = round(yIn/10-50,1);
end