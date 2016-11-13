function [optimalPathX, optimalPathY, psi_goal] = find_path(currX, currY, goalX, goalY, map)
% Written by Kelan Ige & Paulo Lemus
% RobotX Maritime Path Planning
% Requires: Robotics Matlab Toolbox by Peter Corke
% found here: http://www.petercorke.com/RTB/dl-zip.php?file=current/robot-9.10.zip

    currPos = [currX, currY];

    % Create a D* Object using the map
    goal = [goalX, goalY];
    start= currPos;
    ds = Dstar(map);
    ds.plan(goal);

    spath = ds.path(start);
    
    % Publish path information
    optimalPathX = spath(:, 1);
    optimalPathY = spath(:, 2);
    
    % Angle Calculation
    L = length(optimalPathX);
    if(L > 1)
        psi_goal = angle((optimalPathY(L)-optimalPathY(L-1))/(optimalPathX(L)-optimalPathX(L-1)));
    else
        psi_goal = 0;
end
