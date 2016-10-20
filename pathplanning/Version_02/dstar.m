% RobotX Maritime Path Planning
% Requires: Robotics Matlab Toolbox by Peter Corke
% found here: http://www.petercorke.com/RTB/dl-zip.php?file=current/robot-9.10.zip


%% Initialization
% Sub/Pub/Msg
OGridSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
GPSSub = rossubscriber('/GPSLocation', 'sensor_msgs/NavSatFix');
GoalX = rossubscriber('/GoalX', 'std_msgs/Int16');
GoalY = rossubscriber('/GoalY', 'std_msgs/Int16');
OptimalPathxPub = rospublisher('/pathX', 'std_msgs/Int16MultiArray');
OptimalPathyPub = rospublisher('/pathY', 'std_msgs/Int16MultiArray');
PathxMsg = rosmessage(OptimalPathxPub);
PathyMsg = rosmessage(OptimalPathyPub);

% Receive Occupancy grid, gps, goal, and start information from ROS
OGridData = receive(OGridSub, 3);       
GPSData = receive(GPSSub, 3);           % GPSData = GPSSub.LatestMessage;
gX = receive(GoalX, 3);
gY = receive(GoalY, 3);


% Create a D* Object using the map
map = convO2M(OGridData);
goal = [gX, gY];          % TODO: receive start from ROS
start=[0, 0];
ds = Dstar(map);
ds.plan(goal);
spath = ds.path(start);

% Publish path information
PathxMsg.Data = spath(:, 1);
PathyMsg.Data = spath(:, 2);
send(OptimalPathxPub, PathxMsg);
send(OptimalPathyPub, PathyMsg);
