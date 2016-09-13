function four_servo_w_bumper_control()

    % clear workspace and shutdown Matlab instance of ROS if already running
    clear all;
    rosshutdown;

    % Connect to ROS
    rosinit('localhost');
    
    % Pin numbers
    sQ1_pin = 8;
    sQ2_pin = 9;
    sQ3_pin = 10;
    sQ4_pin = 11;
    
    % Setup servos
    [sQ1,sQ2,sQ3,sQ4,sQ1_msg,sQ2_msg,sQ3_msg,sQ4_msg]...
        = servo_setup(sQ1_pin,sQ2_pin,sQ3_pin,sQ4_pin);
    
    % Velocity
    vel = 75;
    
    % Drive straight
    drive_straight(vel,sQ1,sQ2,sQ3,sQ4,...
        sQ1_msg,sQ2_msg,sQ3_msg,sQ4_msg);


end

function [sQ1,sQ2,sQ3,sQ4,sQ1_msg,sQ2_msg,sQ3_msg,sQ4_msg]...
    = servo_setup(sQ1_pin,sQ2_pin,sQ3_pin,sQ4_pin)
    % Sets up servo topics to talk to ROS

    % Enable publishing to servo nodes (publisher name must match topic name in ROS!)
    sQ1 = rospublisher(sprintf('/servo%d',sQ1_pin));   % create Matlab publisher to Q1 servo
    sQ2 = rospublisher(sprintf('/servo%d',sQ2_pin));   % create Matlab publisher to Q2 servo
    sQ3 = rospublisher(sprintf('/servo%d',sQ3_pin));   % create Matlab publisher to Q3 servo
    sQ4 = rospublisher(sprintf('/servo%d',sQ4_pin));   % create Matlab publisher to Q4 servo

    % Create servo message
    sQ1_msg = rosmessage(sQ1);    % create blank ros message for /servo8
    sQ2_msg = rosmessage(sQ2);    % create blank ros message for /servo8
    sQ3_msg = rosmessage(sQ3);    % create blank ros message for /servo8
    sQ4_msg = rosmessage(sQ4);    % create blank ros message for /servo8

end

function drive_straight(vel,sQ1,sQ2,sQ3,sQ4,sQ1_msg,sQ2_msg,sQ3_msg,sQ4_msg)
    % Drives straight at a given speed.  vel should be given from -100% to
    % +100%

    % Edit message
    sQ1_msg.Data = percent2degree(vel);
    sQ2_msg.Data = percent2degree(-vel);
    sQ3_msg.Data = percent2degree(-vel);
    sQ4_msg.Data = percent2degree(vel);

    % Send message 
    send(sQ1,sQ1_msg)
    send(sQ2,sQ2_msg)
    send(sQ3,sQ3_msg)
    send(sQ4,sQ4_msg)
    
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