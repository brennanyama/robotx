function [xObj, yObj] = event_search(OGridData, PoseData, CameraHeadData, CameraColorData)
%This function returns the point of an object as xObj and yObj given angle, from LiDAR and Camera, and color, from Camera. If
%it = 6969, then it did not successfully find an object. This code currently only checks along one ray. Still need to add code to check the entire area of possibility.

% Interpret data
currentX = roundn(PoseData.Pose.Pose.Position.X, -1);   % Extracts current location in OGrid
currentY = roundn(PoseData.Pose.Pose.Position.Y, -1);   % and rounds to nearest tenth
rotZ = PoseData.Pose.Pose.Orientation.Z;                % Angle relative to start, 0. from -1-0-1
phi = CameraHeadData;                                   % Angle of object from center, DEGREES
convFLG = 0
map = readBinaryOccupancyGrid(OGridData);               % Conv. prob. OGrid to Bin. OGrid
convFLG = 1
if rotZ>0
     theta = rotZ*180;                                  % theta is angle relative to start=0*
else if rotZ<0                                          % in degrees
        theta = 360+rotZ*180;
    else
        theta=0;
    end
end

objAngle = theta-phi;
if objAngle>360
     objAngle-360;
end
angleError = 7;                                         % Guessed angle error, +/-


% We now have the x and y location in ogrid, the ogrid, the current
% heading, the current angle of object, and object color.

% Extract objective position data
xRay = currentX;
yRay = currentY;
xEst = currentX;
yEst = currentY;
angle = objAngle;
xObj = 6969;
yObj = 6969;
occupied = 0;

while 1
    for dist = 1:50                             % Only checks up to 5m away
        
        xRay = xRay + 0.1 * cosd(angle);
        yRay = yRay + 0.1 * sind(angle);
        xEst = roundn(xRay, -1);                % The next point in the OGrid along ray
        yEst = roundn(yRay, -1);

        occupied = getOccupancy(map, [xEst, yEst]);
        if occupied                             % If occupied, return and break loop.
            xObj = xEst;
            yObj = yEst;
            break
        end
    end
    if occupied                                 % breaks while loop if condition is met
        break
    end
    % TODO: Add error increments, possibly use Nathan's code to get all
    % points in the cone of possible objects.
end