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
        PoseSub = rossubscriber('/poseupdate', 'geometry_msgs/PoseWithCovarianceStamped');  %Orientation of WAM-V from LiDAR 
        CameraHeadSub = rossubscriber('/cameraHead', 'geometry_msgs/Pose');     %Orientation of WAM-V from camera
        CameraColorSub = rossubscriber('/cameraColor', 'std_msgs/UInt8');       %Color from camera
        GPSSub = rossubscriber('/GPSLocation', 'sensor_msgs/NavSatFix');
        
% Publications
    % Arduino
        mQ1_pub = rospublisher('/motor_q1', 'std_msgs/UInt16');   % create Matlab publisher to Q1 Arduino
        mQ2_pub = rospublisher('/motor_q2', 'std_msgs/UInt16');   % create Matlab publisher to Q2 Arduino
        mQ3_pub = rospublisher('/motor_q3', 'std_msgs/UInt16');   % create Matlab publisher to Q3 Arduino
        mQ4_pub = rospublisher('/motor_q4', 'std_msgs/UInt16');   % create Matlab publisher to Q4 Arduino
    % Loggint
    %    log_pub = rospublisher('/rosout', 'rosgraph_msgs/Log');   % create a publisher for logging system message

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
    % Logging
%        log_msg = rosmessage(log_pub);

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

tic;                % Begin timer, used by motion controller
% while (1)           % Main loop, condition should be changed to ROS is running
    % Receivers
    OGridData = receive(OGridSub);              % receive message from /map topic
    PoseData = receive(PoseSub);                % receive message from /poseupdate topic
    disp('Messages Received');
%send(log_pub, log_msg);
    disp(toc);
%send(log_pub, log_msg);
    % Extract relevant data from ROS messages
    currentX = PoseData.Pose.Pose.Position.X;   % current X position SLAM estimate ROS
    currentY = PoseData.Pose.Pose.Position.Y;   % current Y position SLAM estimate ROS
    rotationZ = PoseData.Pose.Pose.Orientation.Z;   % current heading orientation ROS
    binaryMap = readBinaryOccupancyGrid(OGridData);       % Binary occupancy grid MATLAB
    inflate(binaryMap, 0.02);
    
    matrixOGrid = convB2M(binaryMap);                     % Matrix map DSTAR
    currentPos = convP2M(currentX, currentY, binaryMap);  % Converts current Pos from ros to matrix form
    while (1)
    % Color Recognition
    disp('Starting color recognition.');
    disp(toc);
    [centroid, phi] = color_recognition(vidDevice);
    
    % Event Handler
    disp('Starting event handler.');
    disp(toc);
    [xObj, yObj] = event_location(currentX, currentY, rotationZ, phi, binaryMap);
    
    if (~(isequal(xObj, 6969) || isequal(yObj, 6969)))
        
        xyIn = convP2M(xObj, yObj, binaryMap);
        event_map(xyIn(1), xyIn(2)) = RED;  % CameraColorData;           % Update event map
        event_flag = true;
%        log_msg.msg = 'Event Found!';
%        send(log_pub, log_msg);
        disp('Event Found: ');
        disp(xObj); disp(yObj);
    else
        event_flag = false;
    end

    % Wapoint Descision Logic - sets goalX and goalY coordinates
    disp('Starting Mission Planner.');
    disp(toc);
    if (event_flag || isequal(length(wayX),0))
        [destX, destY] = mission_plan(event_map, currentX, currentY);
        wayX = [destX];                    % Create a new set of waypoints from mission planner
        wayY = [destY];                    % 
        disp('New set of waypoints: ');
        disp(wayX); disp(wayY);
%        log_msg.msg = 'New set of Waypoints.';
%        send(log_pub, log_msg);
    end
    
    % Shortest Path using D*
    disp('Starting path planning.');
    if (and((length(wayX) > 0), (length(pathX) == 0)))          % Arrived at the end of the path and need to process next waypoint
        
        [xOg, yOg] = convP2M(wayX(1), wayY(1), binaryMap);
        
        [pathX, pathY] = find_path(currentPos, xOg, yOg, matrixOGrid);
        disp('Waypoint sent to path planner: ');
        disp(wayX(1)); disp(wayY(1));
        wayX(1) = [];                   % Remove the first waypoint
        wayY(1) = [];                   % Remove the first waypoint
        
    elseif (~isempty(pathX))     %(length(pathX) > 0)                                  % Still have to get to the end of the path
        psi = end_angle(pathX, pathY, rotationZ);  % Arrive at the point and face the direction of the next point
        goal_vars = [pathX(1), pathY(1), psi];            % Set the current coordinates as the waypoint with psi = 0
        disp('Coordinate sent to Motion Controller: ');
        disp(pathX(1)); disp(pathY(1));
        pathX(1) = [];                          % Finished processing the first X coordinate, so remove it
        pathY(1) = [];                          % Finished processing the first Y coordinate, so remove it
    else                % No waypoints or paths
        goal_vars = [currentX, currentY, 0];    % Hold position and heading
    end
    
    goal_vars = [wayX(1), wayY(1), 0];
    % Motion Controller
    disp('Starting motion controller.');
    [time_params,lumped_params,geometry_params,pid_gains,control_tolerances,control_maximums,x,u,up,ui,ud,MC,error,int,behavior] = sim_setup();
    
        % Calculate error matrix
        error = update_error(k,x,goal_vars,control_maximums,error);
        
        % Determine robot behavior based on robot state
        behavior = select_behavior(k,x,control_tolerances,error,behavior);
        
        % Calculate error and PID gains for behavior
        [u,up,ui,ud,error,int] = calculate_gains(k,time_params,pid_gains,u,up,ui,ud,error,int,behavior);
        
        % Publish motor command
        publish_motor_commands(u,MC,k,m_pub,m_msg);
        
%         % Simulate Plant
%         x = plant(k,time_params,x,u,lumped_params,geometry_params);
        
        % Update Loop Variables
        k = int32(mod((k+1), 50000));
    
end

% Release all memory and buffer used
release(hVideoIn); 
release(vidDevice);