function four_servo_w_bumper_control()

    % clear workspace and shutdown Matlab instance of ROS if already running
    clear all;
    rosshutdown;

    % Connect to ROS
    rosinit('localhost');
    
    % Bump sensor pin numbers
    bQ1l = 34;
    bQ1r = 30;
    bQ2l = 32;
    bQ2r = 36;
    bQ3l = 26;
    bQ3r = 24;
    bQ4l = 22;
    bQ4r = 28;
    b_pin = [bQ1l,bQ1r,bQ2l,bQ2r,bQ3l,bQ3r,bQ4l,bQ4r];
       
    % Servo pin numbers
    sQ1_pin = 8;
    sQ2_pin = 9;
    sQ3_pin = 10;
    sQ4_pin = 11;
    s_pin = [sQ1_pin,sQ2_pin,sQ3_pin,sQ4_pin];
    
    % Setup subscription and publishing to bump sensors
    [b_sub,b_pub,b_msg] = bump_pub_sub_setup(b_pin);
    
    % Setup publishing to servo
    [s_pub,s_msg] = servo_pub_setup(s_pin);
    
    % Zero out the bump switches (necessary because ROS is weird like that)
    zero_bump(b_pub,b_msg);
    
    % Drive at 0 vel (stop)
    drive(0,s_pub,s_msg);    
    
    % The ghetto move algorithm...
    dt = 0.01;
    for t = 0:dt:20        
        % Drive
        drive(30,s_pub,s_msg);
        
        if b_sub(1).LatestMessage.Data == 1 |...
                b_sub(2).LatestMessage.Data == 1 |...
                b_sub(3).LatestMessage.Data == 1 |...
                b_sub(4).LatestMessage.Data == 1
            drive(-20,s_pub,s_msg);
            pause(1);
        elseif b_sub(5).LatestMessage.Data == 1 |...
                b_sub(6).LatestMessage.Data == 1 |...
                b_sub(7).LatestMessage.Data == 1 |...
                b_sub(8).LatestMessage.Data == 1
            drive(20,s_pub,s_msg);
            pause(1);
        end
        % Pause
        pause(dt);
    end
    
    % Drive at 0 vel (stop)
    drive(0,s_pub,s_msg);

end

function [b_sub,b_pub,b_msg] = bump_pub_sub_setup(b_pin)
    % Sets up servo topics to talk to ROS

    % Enable subscribing to servo nodes (subscriber name must match topic name in ROS!)
    bQ1l_sub = rossubscriber(sprintf('/bump%d',b_pin(1)));
    bQ1r_sub = rossubscriber(sprintf('/bump%d',b_pin(2)));
    bQ2l_sub = rossubscriber(sprintf('/bump%d',b_pin(3)));
    bQ2r_sub = rossubscriber(sprintf('/bump%d',b_pin(4)));
    bQ3l_sub = rossubscriber(sprintf('/bump%d',b_pin(5)));
    bQ3r_sub = rossubscriber(sprintf('/bump%d',b_pin(6)));
    bQ4l_sub = rossubscriber(sprintf('/bump%d',b_pin(7)));
    bQ4r_sub = rossubscriber(sprintf('/bump%d',b_pin(8)));
    
    % Enable publishing to servo nodes (publisher name must match topic name in ROS!)
    bQ1l_pub = rospublisher(sprintf('/bump%d',b_pin(1)));
    bQ1r_pub = rospublisher(sprintf('/bump%d',b_pin(2)));
    bQ2l_pub = rospublisher(sprintf('/bump%d',b_pin(3)));
    bQ2r_pub = rospublisher(sprintf('/bump%d',b_pin(4)));
    bQ3l_pub = rospublisher(sprintf('/bump%d',b_pin(5)));
    bQ3r_pub = rospublisher(sprintf('/bump%d',b_pin(6)));
    bQ4l_pub = rospublisher(sprintf('/bump%d',b_pin(7)));
    bQ4r_pub = rospublisher(sprintf('/bump%d',b_pin(8)));
    
    % Create bump publisher message
    bQ1l_msg = rosmessage(bQ1l_pub);
    bQ1r_msg = rosmessage(bQ1r_pub);
    bQ2l_msg = rosmessage(bQ2l_pub);
    bQ2r_msg = rosmessage(bQ2r_pub);
    bQ3l_msg = rosmessage(bQ3l_pub);
    bQ3r_msg = rosmessage(bQ3r_pub);
    bQ4l_msg = rosmessage(bQ4l_pub);
    bQ4r_msg = rosmessage(bQ4r_pub);    
    
    % Output array
    b_sub = [bQ1l_sub,bQ1r_sub,bQ2l_sub,bQ2r_sub,...
        bQ3l_sub,bQ3r_sub,bQ4l_sub,bQ4r_sub];
    b_pub = [bQ1l_pub,bQ1r_pub,bQ2l_pub,bQ2r_pub,...
        bQ3l_pub,bQ3r_pub,bQ4l_pub,bQ4r_pub];
    b_msg = [bQ1l_msg,bQ1r_msg,bQ2l_msg,bQ2r_msg,...
        bQ3l_msg,bQ3r_msg,bQ4l_msg,bQ4r_msg];

end

function [s_pub,s_msg] = servo_pub_setup(s_pin)
    % Sets up servo topics to talk to ROS

    % Enable publishing to servo nodes (publisher name must match topic name in ROS!)
    sQ1_pub = rospublisher(sprintf('/servo%d',s_pin(1)));   % create Matlab publisher to Q1 servo
    sQ2_pub = rospublisher(sprintf('/servo%d',s_pin(2)));   % create Matlab publisher to Q2 servo
    sQ3_pub = rospublisher(sprintf('/servo%d',s_pin(3)));   % create Matlab publisher to Q3 servo
    sQ4_pub = rospublisher(sprintf('/servo%d',s_pin(4)));   % create Matlab publisher to Q4 servo

    % Create servo message
    sQ1_msg = rosmessage(sQ1_pub);    % create blank ros message for /servoQ1
    sQ2_msg = rosmessage(sQ2_pub);    % create blank ros message for /servoQ2
    sQ3_msg = rosmessage(sQ3_pub);    % create blank ros message for /servoQ3
    sQ4_msg = rosmessage(sQ4_pub);    % create blank ros message for /servoQ4
    
    % Output array
    s_pub = [sQ1_pub,sQ2_pub,sQ3_pub,sQ4_pub];
    s_msg = [sQ1_msg,sQ2_msg,sQ3_msg,sQ4_msg];

end

function zero_bump(b_pub,b_msg);

    % Send message 
    send(b_pub(1),b_msg(1));
    send(b_pub(2),b_msg(2));
    send(b_pub(3),b_msg(3));
    send(b_pub(4),b_msg(4));
    send(b_pub(5),b_msg(5));
    send(b_pub(6),b_msg(6));
    send(b_pub(7),b_msg(7));
    send(b_pub(8),b_msg(8));

end

function drive(vel,s_pub,s_msg,t)
    % Drives straight at a given speed.  vel should be given from -100% to
    % +100%
    
    % Edit message
    s_msg(1).Data = percent2degree(vel);
    s_msg(2).Data = percent2degree(-vel);
    s_msg(3).Data = percent2degree(-vel);
    s_msg(4).Data = percent2degree(vel);

    % Send message 
    send(s_pub(1),s_msg(1))
    send(s_pub(2),s_msg(2))
    send(s_pub(3),s_msg(3))
    send(s_pub(4),s_msg(4))
    
end

function degree = percent2degree(percent)
    % Converts a velocity given in % to a velocity integer command in
    % degrees (for ROS).  This script also includes min/max PWM margins
    % present in low-cost motor controllers.  Set 'margins' to zero if not
    % needed.  For VEX PWM converters, 45 is a good number.  
    
    % Set margin
    margin = 45;        % min/max PWM margin [degree]
    
    % Check if inout percentage is in correct range.  
    if percent < -100
        disp(['Input velocity of ',num2str(percent),...
            ' is out of bounds. Defaulting to -100']);
        percent = -100;
    elseif percent > 100
        disp(['Input velocity of ',num2str(percent),...
            ' is out of bounds. Defaulting to 100']);
        percent = 100;
    end
    
    % Convert to correct value    
    degree = round(90+(percent*((90-margin)/100)));
    
end
