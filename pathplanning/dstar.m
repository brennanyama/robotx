% RobotX Maritime Path Planning
% Requires: Robotics Matlab Toolbox by Peter Corke
% found here: http://www.petercorke.com/RTB/dl-zip.php?file=current/robot-9.10.zip


% Initialization
pub = 'path';                                   % ROS Topic to publish to
path_pub = rospublisher('Optimal_Path', rostype.std_msgs_Int32MultiArray);

Ogrid = rossubscriber('nav_msgs/OccupancyGrid');

% Get Occupancy Grid from ROS
map = Ogrid.LatestMessage;

% Get Coordinates of Goal from ROS

% Get State from ROS

% Create a D* Object using the map
ds = Dstar(map);
ds.plan(goal);

spath = path(ds, curr_pos);

% Update Path
map = receive(Ogrid);

% Publish the path to ROS
msg = rosmessage(rostype.std_msgs_Int32MultiArray);
msg.data = spath;

send(path_pub, msg);