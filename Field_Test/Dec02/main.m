% Matlab Integration
% University of Hawaii at Manoa
% Fall 2016
% Maritime RobotX
% Author: Kelan Ige

% Define Constants
RED = 1;
GREEN = 2;

% Subscriptions
    % Mission Planner
        OGridSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');     %Occupancy Grid from ROS
        PoseSub = rossubscriber('/poseupdate', 'geometry_msgs/PoseWithCovarianceStamped');  %NEED TO UPDATE TO WORK WITH NEW LOCALIZATION
        CameraHeadSub = rossubscriber('/cameraHead', 'geometry_msgs/Pose');     %Orientation of WAM-V from camera
        CameraColorSub = rossubscriber('/cameraColor', 'std_msgs/UInt8');       %Color from camera
        GPSSub = rossubscriber('/GPSLocation', 'sensor_msgs/NavSatFix');
        
% Publications
    % Arduino
        mQ1_pub = rospublisher('/motor_q1', 'std_msgs/UInt16');   % create Matlab publisher to Q1 Arduino
        mQ2_pub = rospublisher('/motor_q2', 'std_msgs/UInt16');   % create Matlab publisher to Q2 Arduino
        mQ3_pub = rospublisher('/motor_q3', 'std_msgs/UInt16');   % create Matlab publisher to Q3 Arduino
        mQ4_pub = rospublisher('/motor_q4', 'std_msgs/UInt16');   % create Matlab publisher to Q4 Arduino

% Messages
    % Mission Planner
    
    % Arduino
        mQ1_msg = rosmessage(mQ1_pub);    % create blank ros message for /servoQ1
        mQ2_msg = rosmessage(mQ2_pub);    % create blank ros message for /servoQ2
        mQ3_msg = rosmessage(mQ3_pub);    % create blank ros message for /servoQ3
        mQ4_msg = rosmessage(mQ4_pub);    % create blank ros message for /servoQ4
    
        % Output array
        m_pub = [mQ1_pub,mQ2_pub,mQ3_pub,mQ4_pub];
        m_msg = [mQ1_msg,mQ2_msg,mQ3_msg,mQ4_msg];

% Initialize Color Recognition
    % Acquire input video stream
    vidDevice = imaq.VideoDevice('linuxvideo',1); 

    % Set VideoFormat property
    vidDevice.VideoFormat = 'RGB24_640x480';

    % Acquire input video property
    vidInfo = imaqhwinfo(vidDevice); 

    % Output video player
    hVideoIn = vision.VideoPlayer;
    
% Initialize variables
k = 1;
event_map = zeros([1024, 1024]);    % Map used to record events
event_flag = false;                 % Set to true when a new event is detected
wayX = [];                          % Waypoint X coordinates
wayY = [];                          % Waypoint Y coordinates
pathX = [];
pathY = [];
robotRadius = 2.5;

% Initialize PRM object for path finding
prm = robotics.PRM;
prm.NumNodes = 50;                  % Maximum number of waypoints between start and end points
prm.ConnectionDistance = 7;         % Maximum distance between two waypoints in meters

% Initialize Motion Controller Parameters
[time_params,lumped_params,geometry_params,pid_gains,control_tolerances,control_maximums,x,y,snr,rr,meas,u,up,ui,ud,error,int,behavior] = sim_setup();
    

tic;                % Begin timer, used by motion controller


while (1)
    % Receivers
    OGridData = receive(OGridSub);              % receive message from /map topic
    PoseData = receive(PoseSub);                % NEED TO UPDATE WITH NEW LOCALIZATION
    disp('Messages Received');
    phi = 0;
    
    % Update variables from ROS
    disp('Starting occupancy grid to matrix conversion.');
    currentX = PoseData.Pose.Pose.Position.X;   % current X position SLAM estimate ROS
    currentY = PoseData.Pose.Pose.Position.Y;   % current Y position SLAM estimate ROS
    rotationZ = PoseData.Pose.Pose.Orientation.Z;   % current heading orientation ROS
    binaryMap = readBinaryOccupancyGrid(OGridData);       % Binary occupancy grid MATLAB
    inflate(binaryMap, robotRadius);
    prm.Map = binaryMap;                        % set map for path planning
    currentPosition = [currentX, currentY];
    
    % Process Camera Data
    disp('Starting color recognition.');
    [centroid, phi, color] = color_recognition(vidDevice); %added green to this
    disp('Camera Detected Objects:');
    disp([centroid, phi, color]);
    
    % Event Handler
    disp(['Starting event handler.', toc]);
    [xObj, yObj] = event_location(currentX, currentY, rotationZ, phi, binaryMap);
    
    if (~(isequal(xObj, 6969) || isequal(yObj, 6969)))
        
        xyIn = convP2M(xObj, yObj, binaryMap);
        event_map(xyIn(1), xyIn(2)) = RED;  % CameraColorData;           % Update event map
        event_flag = true;
        disp('Event Found: ');
        disp([xObj, yObj]);
    else
        event_flag = false;
    end
    
    % Wapoint Descision Logic - sets goalX and goalY coordinates
    disp('Starting Mission Planner.');
    disp(toc);
    if (event_flag || isequal(length(wayX),0))
        [destX, destY] = mission_plan(event_map, currentX, currentY);
        wayX = destX;                    % Create a new set of waypoints from mission planner
        wayY = destY;                    %
        disp('New set of waypoints: ');
        disp([wayX, wayY]);
    end
    
    % Shortest Path using PRM
    disp('Starting path planning.');
    if (and(~isempty(wayX), isempty(pathX)))          % Arrived at the end of the path and need to process next waypoint
        
        xyOg = convP2M(wayX(1), wayY(1), binaryMap);
        
        PRM_path = findpath(prm, currentPosition, xyOg);
        pathX = PRM_path(:, 1);
        pathY = PRM_path(:, 2);
        show(prm);                      % Show path
        disp('Waypoint sent to path planner: ');
        disp(xyOg(1)); disp(xyOg(2));
        wayX(1) = [];                   % Remove the first waypoint
        wayY(1) = [];                   % Remove the first waypoint
        
    elseif (~isempty(pathX))     %(length(pathX) > 0)                                  % Still have to get to the end of the path
        psi = end_angle(pathX, pathY, rotationZ);  % Arrive at the point and face the direction of the next point
        
        pos = convM2P(pathX(1), pathY(1), binaryMap);
        goal_vars = [pos(1), pos(2), psi];            % Set the current coordinates as the waypoint with psi = 0
        disp('Coordinate sent to Motion Controller: ');
        disp(pathX(1)); disp(pathY(1));
        pathX(1) = [];                          % Finished processing the first X coordinate, so remove it
        pathY(1) = [];                          % Finished processing the first Y coordinate, so remove it
    else                % No waypoints or paths
        goal_vars = [currentX, currentY, 0];    % Hold position and heading
    end
    
    % Motion Controller
    disp('Starting motion controller.');

    % Calculate error matrix
    error = update_error(k,x,meas,goal_vars,error);
    
    % Determine robot behavior based on robot state
    behavior = select_behavior(k,x,control_tolerances,error,behavior);
    
    % Calculate error and PID gains for behavior
    [u,up,ui,ud,error,int] = calculate_gains(k,time_params,pid_gains,u,up,ui,ud,error,int,behavior);
    
    % Simulate Plant
    [x,y] = plant(k,time_params,x,y,u,lumped_params,geometry_params);
    
    % Simulate sensor inputs
    meas = sensors(y, meas, k, time_params, snr, rr);
    
    % Convert controller output [N] to PWM signal to motors
    mQ1_msg.Data = n2pwmF(u(1, k));
    mQ2_msg.Data = n2pwmF(u(2, k));
    mQ3_msg.Data = n2pwmR(u(3, k));
    mQ4_msg.Data = n2pwmR(u(4, k));
    
    % Publish motor command
    publish_motor_commands(m_pub,m_msg);
    disp('Motor Commands:'); disp([mQ1_msg.Data, mQ2_msg.Data, mQ3_msg.Data, mQ4_msg.Data]);
    
    % Update Loop Variables
    k = int32(mod(k, time_params(3))+1);
end

% Release all memory and buffer used
release(hVideoIn); 
release(vidDevice);
