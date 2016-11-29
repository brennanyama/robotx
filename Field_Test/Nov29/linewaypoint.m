function [wayX, wayY] = linewaypoint(x, y, m, D, N)
%Returns a sequence of N waypoints along a line, given a starting X and Y
%coordinate, slope, Y-intercept (b), and the distance D between waypoints.
    wayX = zeros([1, N]);
    wayY = zeros([1, N]);
    
    wayX(1) = x;
    wayY(1) = y;
    
    theta = atan(m);
    
    for i = 2:N
        wayX(i) = wayX(i-1)+D*cos(theta);
        wayY(i) = wayY(i-1)+D*sin(theta);
    end
end