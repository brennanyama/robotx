% RobotX Maritime Path Planning
% Requires: Robotics Matlab Toolbox by Peter Corke
% found here: http://www.petercorke.com/RTB/dl-zip.php?file=current/robot-9.10.zip


%% Initialization
% Subscribers
OGridSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
GPSSub = rossubscriber('/GPSLocation', 'sensor_msgs/NavSatFix');
GoalX = rossubscriber('/GoalX', 'std_msgs/Int16');
GoalY = rossubscriber('/GoalY', 'std_msgs/Int16');
PoseSub = rossubscriber('/poseupdate', 'geometry_msgs/PoseWithCovarianceStamped');

% Publishers
OptimalPathxPub = rospublisher('/pathX', 'std_msgs/Int16MultiArray');
OptimalPathyPub = rospublisher('/pathY', 'std_msgs/Int16MultiArray');

% Messages
PathxMsg = rosmessage(OptimalPathxPub);
PathyMsg = rosmessage(OptimalPathyPub);

% Receivers
OGridData = receive(OGridSub); 
PoseData = receive(PoseSub);      
% GPSData = receive(GPSSub);           % GPSData = GPSSub.LatestMessage;
% gX = receive(GoalX);
% gY = receive(GoalY);

% Interpret data
currX = roundn(PoseData.Pose.Pose.Position.X, -1);
currY = roundn(PoseData.Pose.Pose.Position.Y, -1);
currPos = [currX, currY];


% Create a D* Object using the map
[map, currPos] = convO2M(OGridData, currPos)
goal = [570, 275];          % TODO: receive start from ROS
start= currPos;		    % [512, 512]
MapConversionFinished=1
ds = Dstar(map);
DSMapFinished=1
ds.plan(goal);

spath = ds.path(start);

spath


% Publish path information
PathxMsg.Data = spath(:, 1);
PathyMsg.Data = spath(:, 2);
send(OptimalPathxPub, PathxMsg);
send(OptimalPathyPub, PathyMsg);
