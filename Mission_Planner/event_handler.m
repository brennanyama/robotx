%% Event_Handler
% Paulo Lemus
% This node is a part of the event handler.
% 
% ROS
clear
clc
%rosinit
 
% Subscribers
OGridSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
PoseSub = rossubscriber('/poseupdate', 'geometry_msgs/PoseWithCovarianceStamped');
CameraHeadSub = rossubscriber('/cameraHead', 'geometry_msgs/Pose');
CameraColorSub = rossubscriber('/cameraColor', 'std_msgs/UInt8');
 
% Receivers
OGridData = receive(OGridSub);
PoseData = receive(PoseSub);
CameraHeadData = receive(CameraHeadSub);
CameraColorData = receive(CameraColorSub);

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
angleError = 8;                                         % Guessed angle error, +/-


% We now have the x and y location in ogrid, the ogrid, the current
% heading, the current angle of object, and object color.

% Extract objective position data
xRay = currentX;
yRay = currentY;
xEst = currentX;
yEst = currentY;
xObj = 6969;                % IF this remains as 6969, then we did not find a valid object.
yObj = 6969;
occupied = 0;

% These array arrayMix is an array that will search cental out.
arrayL = [objAngle:0.5:objAngle+angleError];
arrayR = [objAngle-0.5:-0.5:objAngle-angleError];
arrayMix(1:2:length(arrayL)*2) = arrayL;
arrayMix(2:2:length(arrayR)*2) = arrayR;

% We now find the most likely object along our angle for the object. If we
% do not find any along the most likely line, we search from middle out to
% error.


while 1
    for element = 1:length(arrayMix)
        
        for dist = 1:50                             % Only checks up to 5m away

            xRay = xRay + 0.1 * cosd(arrayMix(element));
            yRay = yRay + 0.1 * sind(arrayMix(element));
            xEst = roundn(xRay, -1);                % The next point in the OGrid along ray
            yEst = roundn(yRay, -1);

            occupied = getOccupancy(map, [xEst, yEst]);
            if occupied                             % If occupied, return and break loop.
                
                xObj = xEst;
                yObj = yEst;
                objLocation = [xObj, yObj];
                break;
            end
            if occupied
                occupied = 0;
                break;
            end
        end
        % break?
    end
    % TODO: Add error increments, possibly use Nathan's code to get all
    % points in the cone of possible objects.
end

% xObj and yObj are the point of an object of the given angle and color. If
% it = 6969, then it did not successfully find an object. This code
% currently only checks along one ray. Still need to add code to check the
% entire area of possibility.
    


