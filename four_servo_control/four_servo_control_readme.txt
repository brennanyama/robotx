Servo control terminal commands readme.  This assumes ROS Indigo is installed your 14.04 Ubuntu machine.

1. Install arduino IDE (http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).  Plug in arduino and upload four_servo_control.ino firmware to the arduino (if not already uploaded).

2. Enter terminal commands: 

  $ roscore 
  (new terminal)
  $ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0	// might need to chance the ACM to USB...check the Arduino IDE
  If desired, the servo topics can be published to manually using the command below,
  $ rostopic pub servo8 std_msgs/UInt16 <angle>			// angle goes from 0 to 180, change the number (8-11) after servo to change the servo
  
3. Install Matlab.  Run the Matlab script four_servo_control.m.  Matlab will create a Matlab node, and publish to aforementioned topics.  Values in script can be edited.
