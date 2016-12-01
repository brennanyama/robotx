function psi = end_angle(pathX, pathY, rotZ)
%This function calculates the angle to arrive at a point so that it is
%facing the next point to go to.
    if (length(pathX) > 1)
        psi = rotZ - atan((pathY(2)-pathY(1))/(pathX(2)-pathX(1))); % Adjust heading relative to current heading
    else
        psi = 0;        % Maintain same heading because the next point is the end of the path
    end
end