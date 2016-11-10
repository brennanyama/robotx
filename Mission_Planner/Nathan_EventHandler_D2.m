%Nathan Park
%Note: camera topics and nodes are not up yet.
rosshutdown
rosinit

% Subscribers
OGridSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid'); %Occupancy grid from lidar
PoseSub = rossubscriber('/poseupdate', 'geometry_msgs/PoseWithCovarianceStamped');%orientation of boat according lidar.
CameraHeadSub = rossubscriber('/cameraHead', 'geometry_msgs/Pose');%orientation of boat according to camera
CameraColorSub = rossubscriber('/cameraColor', 'std_msgs/UInt8');%color from camera
CameraObjAngsub = rossubscriber('/cameraObj','std_msgs/ObjA'); %angle of the object WRT to boat heading 
                                                               %according to camera
 
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
% We now have the x and y location, and heading of boat in ogrid.  

% Extract objective position data
xRay = currentX;        
yRay = currentY;
xEst = currentX;
yEst = currentY;
angle = objAngle;
xObj = 6969;
yObj = 6969;

OBJ = zeros(60)
occupied = 0;

ang_cone1 = CameraHeadSub +35           %probability cone from camera: upper bound angle WRT x-axis
ang_cone0 = CameraHeadSub - 35          %lower bound WRT x-axis
a = ang_cone1 %iterated ray angle 

%Nested loop that will scan a probability triangle for object points

while 1
    %We start at the boundary with the greatest angle from the horizontal axis.
    %We take points along a ray with the same orientation as the boundary.
    %Then de-increment the angle by 5 until we reach the lower boundary of
    %the triangle.
    for ang_cone1:ang_cone0  
        for 1:50
            xRay = xRay + 0.1 * cosd(a);
        yRay = yRay + 0.1 * sind(a);
        xEst = roundn(xRay, -1);                % The next point in the OGrid along ray
        yEst = roundn(yRay, -1);

        occupied = getOccupancy(map, [xEst, yEst]); %checks if value inside index is 0??? 
        if occupied                             % If occupied, return and break loop.
            xObj = xEst;
            yObj = yEst;
            break
        end
    if occupied                                 % breaks while loop if condition is met
        yaw = (a-theta;                         %angle of the object wrt to boat's heading according to lidar
        if(abs(yaw-phi) 
       OBJ( xEst,yEst) =  CameraColorSub;
        break
        else
            break
        end   
     end
     end
  a = a-5 
    end
end