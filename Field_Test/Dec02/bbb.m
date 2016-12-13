rosinit;
% Map reset
    RESETSub = rossubscriber('/tits', 'std_msgs/UInt8');            % MAP Reset Sub
    GPSSub = rossubscriber('/fix', 'sensor_msgs/NavSatFix');
    IMUSub = rossubscriber('/top_imu', 'sensor_msgs/Imu');

% Publishers
    OdoPub = rospublisher('/BrennansOdo', 'geometry_msgs/PoseWithCovarianceStamped');
    %OdoMsg = rosmessage(OdoPub);
    OdoMsg = rosmessage('geometry_msgs/PoseWithCovarianceStamped');
    OdoMsg.Header.FrameId = 'base_link';
    
% Wait for reset
    RESETmsg = receive(RESETSub);
    tftree = rostf;
    if (isequal(RESETmsg.Data, 1))
	GPSmsg = receive(GPSSub);
        IMUmsg = receive(IMUSub);
    end
    
    lat = GPSmsg.Latitude;
    lon = GPSmsg.Longitude;
    ANG = quat2eul([IMUmsg.Orientation.W, IMUmsg.Orientation.X, IMUmsg.Orientation.Y, IMUmsg.Orientation.Z])
    
    heading = atan2(ANG(3), ANG(2))
    
    llo = [lat, lon]
    
    while(1)
        GPSmsg = receive(GPSSub);
	flag = GPSmsg.Status.Status;
    	lat = GPSmsg.Latitude
    	lon = GPSmsg.Longitude
        lla = [lat, lon, 0]
    
        flatearth_pos = lla2flat(lla, llo, heading, 0)
        
        OdoMsg.Pose.Pose.Position.X = flatearth_pos(1);
        OdoMsg.Pose.Pose.Position.Y = flatearth_pos(2);
        OdoMsg.Pose.Pose.Orientation.X = IMUmsg.Orientation.X;
	OdoMsg.Pose.Pose.Orientation.Y = IMUmsg.Orientation.Y;
	OdoMsg.Pose.Pose.Orientation.Z = IMUmsg.Orientation.Z;
	OdoMsg.Pose.Pose.Orientation.W = IMUmsg.Orientation.W;

        if (isequal(flag,0))
            tfpt = transform(tftree, 'odom', OdoMsg);
	        send(OdoPub, OdoMsg);
        end
    end
