% four_servo_control

% clear workspace and shutdown Matlab instance of ROS if already running
clear all;
rosshutdown;

% Connect to ROS
rosinit('localhost');

% Enable publishing to servo nodes
servo8 = rospublisher('/servo8');   % create Matlab publisher to /servo8
servo9 = rospublisher('/servo9');   % create Matlab publisher to /servo8
servo10 = rospublisher('/servo10');   % create Matlab publisher to /servo8
servo11 = rospublisher('/servo11');   % create Matlab publisher to /servo8

% Create message
servo8_msg = rosmessage(servo8);    % create blank ros message for /servo8
servo9_msg = rosmessage(servo9);    % create blank ros message for /servo8
servo10_msg = rosmessage(servo10);    % create blank ros message for /servo8
servo11_msg = rosmessage(servo11);    % create blank ros message for /servo8

% Edit message
n = 105;
servo8_msg.Data = n;
servo9_msg.Data = n;
servo10_msg.Data = n;
servo11_msg.Data = n;

% Send message 
send(servo8,servo8_msg)
send(servo9,servo9_msg)
send(servo10,servo10_msg)
send(servo11,servo11_msg)
