%Nathan Park
%Note: camera topics and nodes are not up yet.
rosshutdown
rosinit

% SUBSCRIBERS
%posesub = rossubscriber('/imu', 'sensor_msgs/Imu'); ?Confirm: not used to get pose?  
pose2sub = rossubscriber('/poseupdate', 'geometry_msgs/PoseWithCovarianceStamped') 
                                        %compact message consist of an
                                        %(x,yz) point, and quaternion orientation.
                                         
                                                                                        
mapsub = rossubscriber('/map','nav_msgs/OccupancyGrid'); %subscribing to map topic 
camerasub = rossubscriber('/camera', '_msgs/heading'); %subscribe to camera topic for heading !!!NEED TOPIC AND MSG NAME
colorsub = rosssubscriber('/camera', '_msgs/color'); %subscribe to camera topic for color NEED TOPIC AND MSG NAME!

% RECEIVE DATA
camerascan = recieve(camerasub) %recieve heading of objects from camera   
colorscan = receive(colorsub) %recieve color represented by an integer 
mapscan = receive(mapsub) %get set of points after next Lidar scan. 

%Map to be analyzed
map = readBinaryOccupancyGrid('nav_msgs/OccupancyGrid', threshold); % converting occupancy grid to binary.

Occupied = map == 1;
n = nume1(Occupied)
[row,col] = find(map>0,n);

for i = 0:n
    i+1
    %x0=x position of boat, y0 = y position of boat 
    xi = row(i)
    yi = col (i)
    %QUATERNION TO RAD CONVERSION OF POSE ORIENTATION
    %assuming angle 0 is north, angle radians 
    phi %function for object heading calculated by camera node
    yaw = (pi/2-arctan((yi-y0)/(xi-x0)))+ pose_angle   
    %figure out what angle this will be, center line of lidar aand camera,
    %angle of object?
 if abs(phi - yaw)< 10 %if difference between lidar and camera angle is less than degrees. 
     if mod((phi + yaw),70) == 0  %still working out linking coordinates to right heading. I got it to check they are in the same field of view. 
        
        heading = phi
        color %integer representing color assoicated with an angle from camera
        %event types: 'object1' represents object of first priority, '2'
        %second piority ect.
        obstical = 0
        totem1 = 1 
        totem2 = 2
        totem3 = 3
        
        buoy1 = 1.1
        buoy2 = 2.2
        buoy3 = 3.3
        
        switch color 
            case 1 0
                event_type = obstical 
            case 2 
            case 3
            case 4
        end
    else %not yet sure how to compensate if heading readings are not matching up
     end
 end
end