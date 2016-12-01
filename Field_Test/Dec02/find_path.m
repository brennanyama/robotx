function [optimalPathX, optimalPathY] = find_path(currPos, goalX, goalY, map)
% Written by Kelan Ige & Paulo Lemus
% RobotX Maritime Path Planning
% Requires: Robotics Matlab Toolbox by Peter Corke
% found here: http://www.petercorke.com/RTB/dl-zip.php?file=current/robot-9.10.zip
    

    % Create a D* Object using the map
    goal = [goalX, goalY];
    start= currPos;
    ds = Dstar(map);
    ds.plan(goal);

    spath = ds.path(start);
    
    % Publish path information
    optimalPathX = spath(:, 1);
    optimalPathY = spath(:, 2);
end
