% Matlab Integration
% University of Hawaii at Manoa
% Fall 2016
% Maritime RobotX
% Author: Kelan Ige

% Define Color Integer Constants
RED = 1;
GREEN = 2;
BLUE = 3;
YELLOW = 4;

% Define Task Number
TASKNUM = 0;        % Number of task being attempted
DISTANCE = 30;

% Subscriptions
    % Mission Planner
        OGridSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');     %Occupancy Grid from ROS
        OdoSub = rossubscriber('/odom', 'nav_msgs/Odometry');  %State estimate
%        GPSSub = rossubscriber('/fix', 'sensor_msgs/NavSatFix');
%        WAMVSub = rossubscriber('/status_pub', 'std_msgs/UInt16');
%       OdoSub = rossubscriber('/top_imu', 'sensor_msgs/Imu');  %State estimate
        
% Publication
    % Arduino
        mQ1_pub = rospublisher('/motor_q1', 'std_msgs/UInt16');   % create Matlab publisher to Q1 Arduino
        mQ2_pub = rospublisher('/motor_q2', 'std_msgs/UInt16');   % create Matlab publisher to Q2 Arduino
        mQ3_pub = rospublisher('/motor_q3', 'std_msgs/UInt16');   % create Matlab publisher to Q3 Arduino
        mQ4_pub = rospublisher('/motor_q4', 'std_msgs/UInt16');   % create Matlab publisher to Q4 Arduino
        
%        arm_pub = rospublisher('/arm_start', 'std_msgs/Int8');    % create Matlab publisher to Arm Arduino

% Messages
    % Arduino
        mQ1_msg = rosmessage(mQ1_pub);    % create blank ros message for /motor_q1
        mQ2_msg = rosmessage(mQ2_pub);    % create blank ros message for /motor_q2
        mQ3_msg = rosmessage(mQ3_pub);    % create blank ros message for /motor_q3
        mQ4_msg = rosmessage(mQ4_pub);    % create blank ros message for /motor_q4

%        arm_msg = rosmessage(arm_pub);    % create blank ros message for /arm_start
        % Output array
        m_pub = [mQ1_pub,mQ2_pub,mQ3_pub,mQ4_pub];
        m_msg = [mQ1_msg,mQ2_msg,mQ3_msg,mQ4_msg];

% Initialize Color Recognition
    % Acquire input video stream
    vidDevice = imaq.VideoDevice('linuxvideo', 2); 

    % Set VideoFormat property
    vidDevice.VideoFormat = 'RGB24_640x480';

    % Acquire input video property
    vidInfo = imaqhwinfo(vidDevice); 
    
% Initialize variables
k = 1;
event_map = zeros([1024, 1024]);    % Map used to record events
event_flag = false;                 % Set to true when a new event is detected
wayX = [];                          % Waypoint X coordinates
wayY = [];                          % Waypoint Y coordinates
pathX = [];
pathY = [];
goal_vars = [];
robotRadius = 0.25;

% Initialize PRM object for path finding
prm = robotics.PRM;
prm.NumNodes = 750;                  % Maximum number of waypoints between start and end points
prm.ConnectionDistance = 8;         % Maximum distance between two waypoints in meters

% Initialize Motion Controller Parameters
[time_params,lumped_params,geometry_params,pid_gains,control_tolerances,x_plant,y,snr,rr,meas,u,up,ui,ud,error,int,behavior] = sim_setup();
x_real = zeros(size(x_plant));    
k = 1;
event_map = zeros([1024, 1024]);    % Map used to record events
event_flag = false;                 % Set to true when a new event is detected
wayX = [];                          % Waypoint X coordinates
wayY = [];                          % Waypoint Y coordinates
pathX = [];
pathY = [];
task2head = [];
robotRadius = 0.25;

% To test Waypoint Controller
%distance = 12;            % distance [meters] to move forward
%goal_vars = []; %input('Hold Positon (x, y, psi): ');		% commented out all goal_vars in while loop!
tic;                % Begin timer, used by motion controller
t_old = 0;
t_new = 0;

while (1)
    t_new = toc;
    dt = t_new - t_old;
    
    disp(['Loop iteration: ', num2str(k)]);
    % Receivers
    OGridData = receive(OGridSub);              % receive message from /map topic
    OdoData = receive(OdoSub);                	% using new odometry
%    WAMVData = receive(WAMVSub);		% WAMV status
    disp('Messages Received');
%    phi = 0;
    
    % Update variables from ROS
    currentX = OdoData.Pose.Pose.Position.X;   % current X position SLAM estimate ROS
    currentY = OdoData.Pose.Pose.Position.Y;   % current Y position SLAM estimate ROS
    quaternionX = OdoData.Pose.Pose.Orientation.X;   % current heading orientation ROS
    quaternionY = OdoData.Pose.Pose.Orientation.Y;   % current heading orientation ROS
    quaternionZ = OdoData.Pose.Pose.Orientation.Z;   % current heading orientation ROS
    quaternionW = OdoData.Pose.Pose.Orientation.W;   % current heading orientation ROS
    twistX = OdoData.Twist.Twist.Linear.X;          % current X velocity
    twistY = OdoData.Twist.Twist.Linear.Y;
    twistZ = OdoData.Twist.Twist.Linear.Z;
    binaryMap = readBinaryOccupancyGrid(OGridData);       % Binary occupancy grid MATLAB
    inflate(binaryMap, robotRadius);
    prm.Map = binaryMap;                        % set map for path planning
    currentPosition = [currentX, currentY];
    rot_angles = quat2eul([quaternionW, quaternionX, quaternionY, quaternionZ]); % returns rotations in the Z, Y, X 
    rotationZ = rot_angles(1);
    disp('Odometry Position: ');
    disp(currentPosition);
    disp(rotationZ);

    % Update robot state
        x_real(1, k) = currentX;
        x_real(2, k) = currentY;
        x_real(3, k) = rotationZ; 
        x_real(4, k) = twistX;
        x_real(5, k) = twistY;
        x_real(6, k) = atan2(twistY, twistX);

    % Process Camera Data
   disp('Starting color recognition.');
   [centroid, phi, color] = color_recognition(vidDevice); %added green to this
   disp('Camera Detected Objects:');
   disp([centroid, phi, color]);
    
    % Event Handler
    disp('Starting event handler.');
    while   (~isempty(phi))
        [xObj, yObj] = event_location(currentX, currentY, rotationZ, phi(1), binaryMap);
    
        if (~(isequal(xObj, 6969) || isequal(yObj, 6969)))
        
            xyIn = convP2M(xObj, yObj, binaryMap);
            event_map(xyIn(1), xyIn(2)) = color(1);          % Update event map
            event_flag = true;
            disp('Event Found: ');
            disp([xObj, yObj]);
        end
        
        centroid(1, :) = [];            % Remove processed colors
        phi(1) = [];
        color(1) = [];
    end

    % Wapoint Descision Logic - sets goalX and goalY coordinates
   disp('Starting Mission Planner.');
   disp(toc);
    if (event_flag || isempty(wayX))
        [destX, destY] = mission_plan(event_map, currentX, currentY, rotationZ, TASKNUM);
        wayX = destX;                    % Create a new set of waypoints from mission planner
        wayY = destY;                    %
        disp('New set of waypoints: ');
        disp([wayX, wayY]);
    end
	    
    % Shortest Path using PRM
 %   disp('Starting path planning.');
 %   if (and(~isempty(wayX), isempty(pathX)))          % Arrived at the end of the path and need to process next waypoint
        
	% Enforce minimum distance between waypoints
%	while (sqrt((currentX-wayX(1))^2+(currentY-wayY(1))^2) < 5)
%		wayX(1) = [];
%		wayY(1) = [];
%	end
%        xyOg = convP2M(wayX(1), wayY(1), binaryMap);
%        xyOg = limit_waypoint(xyOg);
%        PRM_path = findpath(prm, currentPosition, xyOg);
%	if (~isempty(PRM_path))
%	        pathX = PRM_path(:, 1);
%        	pathY = PRM_path(:, 2);
%	end
%        disp('Waypoint sent to path planner: ');
%        disp(xyOg(1)); disp(xyOg(2));
%       wayX(1) = [];                   % Remove the first waypoint
%       wayY(1) = [];                   % Remove the first waypoint
%      goal_vars = [PRM_path(1,1), PRM_path(1,2), 0];
%	pathX(1) = [];
%	pathY(1) = [];
%    elseif (~isempty(pathX))                       % Still have to get to the end of the path
%        psi = end_angle(pathX, pathY, rotationZ);  % Arrive at the point and face the direction of the next point    
%        pos = convM2P(pathX(1), pathY(1), binaryMap);
%        goal_vars = [pos(1), pos(2), psi];            % Set the current coordinates as the waypoint with psi = 0
%        disp('Coordinate sent to Motion Controller: ');
%        disp(pathX(1)); disp(pathY(1));
%        pathX(1) = [];                          % Finished processing the first X coordinate, so remove it
%        pathY(1) = [];                          % Finished processing the first Y coordinate, so remove it
%    else                % No waypoints or paths
%        goal_vars = [currentX, currentY, 0];    % Hold position and heading
%   end

    % Motion Controller
    disp('Starting motion controller.');
    
    if (isempty(goal_vars))
        if (isequal(TASKNUM, 0))
            goal_vars = [DISTANCE*cos(rotationZ), DISTANCE*sin(rotationZ), 0];
        elseif (isequal(TASKNUM, 6))
            task2head = rotationZ;
            goal_vars = [wayX(1), wayY(1), task2head];
            wayX(1) = [];
            wayY(1) = [];
        end
    end
    % Display Goal Coordinate
	disp('Goal Coordinates: ');
	disp(goal_vars);

    % Calculate error matrix
    error(:, k) = update_error(k,x_real,goal_vars);
    
    % Determine robot behavior based on robot state
    behavior(:, k) = select_behavior(k,x_real,control_tolerances,error);

    % Calculate error and PID gains for behavior
    [u,up,ui,ud,int] = calculate_gains(k,dt,pid_gains,u,up,ui,ud,error,int,behavior);
    
    % Simulate Plant
    [x_plant,y] = plant(k,dt,time_params,x_plant,y,u,lumped_params,geometry_params);
    
    % Simulate sensor inputs
    meas = sensors(y, meas, k, dt, time_params, snr, rr);
    
    % Convert controller output [N] to PWM signal to motors
    mQ1_msg.Data = n2pwmF(u(1, k));
    mQ2_msg.Data = n2pwmF(u(2, k));
    mQ3_msg.Data = n2pwmR(u(3, k));
    mQ4_msg.Data = n2pwmR(u(4, k));
    
    if (isequal(TASKNUM, 6))
        % 90 degree right strafe
        mQ1_msg.Data = 0;
        mQ2_msg.Data = 99;
        mQ3_msg.Data = 0;
        mQ4_msg.Data = 99;
    end
    % Publish motor command
    send(m_pub(1), m_msg(1));
    send(m_pub(2), m_msg(2));
    send(m_pub(3), m_msg(3));
    send(m_pub(4), m_msg(4));
    disp([u(1, k), u(2, k), u(3, k), u(4, k)]);
    disp('Motor Commands:'); disp([mQ1_msg.Data, mQ2_msg.Data, mQ3_msg.Data, mQ4_msg.Data]);
    
    % Update Loop Variables
    k = int32(mod(k, time_params(3))+1);
   
    % Reset logic flags
    event_flag = false;
    
    % Update time
    t_old = t_new;
end

% Release all memory and buffer used
release(hVideoIn);
release(vidDevice);
