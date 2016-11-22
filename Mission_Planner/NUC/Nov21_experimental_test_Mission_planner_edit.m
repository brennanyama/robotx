R = rosrate(1);              % Set rate to 1Hz
RED = 1;                     % Set constant for RED buoys
GREEN = 2;                   % Set constant for GREEN buoys

% Subscriptions
    % Mission Planner
        OGridSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');     %Occupancy Grid from ROS
        PoseSub = rossubscriber('/poseupdate', 'geometry_msgs/PoseWithCovarianceStamped');  %Orientation of WAM-V from LiDAR 
        CameraHeadSub = rossubscriber('/cameraHead', 'geometry_msgs/Pose');     %Orientation of WAM-V from camera
        CameraColorSub = rossubscriber('/cameraColor', 'std_msgs/UInt8');       %Color from camera
        GPSSub = rossubscriber('/fix', 'sensor_msgs/NavSatFix');
        IMUSub = rossubscriber('/imu', 'sensor_msgs/Imu');
        
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
    
% Receivers
    OGridData = receive(OGridSub);              % receive message from /map topic
    disp('Messages Received');

    % Extract relevant data from ROS messages
    IMUData = receive(IMUSub);
    GPSData = receive(GPSSub);
    PoseData = receive(PoseSub);                % receive message from /poseupdate topic
    currentX = PoseData.Pose.Pose.Position.X;   % current X position SLAM estimate ROS
    currentY = PoseData.Pose.Pose.Position.Y;   % current Y position SLAM estimate ROS
    rotationZ = PoseData.Pose.Pose.Orientation.Z;   % current heading orientation ROS
    disp('Starting occupancy grid to matrix conversion.');
    binaryMap = readBinaryOccupancyGrid(OGridData);       % Binary occupancy grid MATLAB
    inflate(binaryMap, 0.02);
    matrixOGrid = convB2M(binaryMap);                     % Matrix map DSTAR
    currentPos = convP2M(currentX, currentY, binaryMap);  % Converts current Pos from ros to matrix form
    disp('Finished Occupancy grid conversion.');

event_map = zeros([1024, 1024]);    % Map used to record events

while (1)
    % Display Position Data
    disp('SLAM: ');
    showdetails(PoseData);
    disp('GPS: ');
    showdetails(GPSData);
    disp('IMU: ');
    showdetails(IMUData);
    
    % Process Camera Data
    [centroid, phi, color] = color_recognition(vidDevice); %added green to this
    disp('Camera Detected Objects:');
    disp(centroid); disp(phi); disp(color);
    
    % Event Handler
    disp('Starting event handler.');
    while(~isempty(phi))
        [xObj, yObj] = event_location(currentX, currentY, rotationZ, phi(1), binaryMap);
        
        if (~(isequal(xObj, 6969) || isequal(yObj, 6969)))
            xyIn = convP2M(xObj, yObj, binaryMap);
            event_map(xyIn(1), xyIn(2)) = color(1);          % Update event map
            disp('Event Found: ');
            disp(xObj); disp(yObj); disp(color(1));
        end
        % Remove processed Detected Color
        centroid(1) = [];
        phi(1) = [];
        color(1) = [];
    end
    
    [red_locX, red_locY] = find(isequal(event_map, RED));           % Location of all known red buoys returned as column vector
    [green_locX, green_locY] = find(isequal(event_map, GREEN));     % Location of all known green buoys returned as column vector
    
    if(~isempty(red_locX))
        if(isempty(green_locX)) %no green, rotate cw to search
            mQ1_msg.Data = 25; %approximate 50% duty cycle
            mQ2_msg.Data = 75;
            mQ3_msg.Data = 75;
            mQ4_msg.Data = 25;
        elseif(~isempty(green_centroid) && (abs(sqrt((currentX-red_locX(1))^2+(currentY-red_locY)^2)-sqrt((currentX-green_locX(1))^2+(currentY-green_locY)^2)) < 0.3))  % if distance between two buoys is about the same
            mQ1_msg.Data = 99;
            mQ2_msg.Data = 99;
            mQ3_msg.Data = 99;
            mQ4_msg.Data = 99;
        else %green is there but not in line, continue search
            mQ1_msg.Data = 25;
            mQ2_msg.Data = 75;
            mQ3_msg.Data = 75;
            mQ4_msg.Data = 25;
        end
    elseif(~isempty(green_locX) && isempty(red_locX)) %see green, but no red rotate ccw
        mQ1_msg.Data = 75;
        mQ2_msg.Data = 25;
        mQ3_msg.Data = 25;
        mQ4_msg.Data = 75;
    else %see nothing rotate cw
        mQ1_msg.Data = 25;
        mQ2_msg.Data = 75;
        mQ3_msg.Data = 75;
        mQ4_msg.Data = 25;
    end
    
    % Publish to ROS
    send(m_pub(1),m_msg(1));
    send(m_pub(2),m_msg(2));
    send(m_pub(3),m_msg(3));
    send(m_pub(4),m_msg(4));
    
    % Display in MATLAB
    disp('Motor Values: ');
    disp(mQ1_msg.Data); disp(mQ2_msg.Data); disp(mQ3_msg.Data); disp(mQ4_msg.Data);
    
    % Update Position Data
    PoseData = receive(PoseSub);                % receive message from /poseupdate topic
    currentX = PoseData.Pose.Pose.Position.X;   % current X position SLAM estimate ROS
    currentY = PoseData.Pose.Pose.Position.Y;   % current Y position SLAM estimate ROS
    rotationZ = PoseData.Pose.Pose.Orientation.Z;   % current heading orientation ROS
    disp('Updating Occupancy Grid.');
    binaryMap = readBinaryOccupancyGrid(OGridData);       % Binary occupancy grid MATLAB
    inflate(binaryMap, 0.02);
    matrixOGrid = convB2M(binaryMap);                     % Matrix map DSTAR
    currentPos = convP2M(currentX, currentY, binaryMap);  % Converts current Pos from ros to matrix form
    disp('Finished Updating occupancy Grid.');

    GPSData = receive(GPSSub);
    IMUData = receive(IMUSub);
    
    waitfor(R);
end

% Release all memory and buffer used
release(hVideoIn);
release(vidDevice);
