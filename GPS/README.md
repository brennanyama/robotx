# Helpful links:
Troubleshooting: http://www.sustainablenetworks.org/CIS623/?page_id=142
GPS Shield Kit: https://learn.sparkfun.com/tutorials/gps-shield-hookup-guide
Library for GPS: http://arduiniana.org/libraries/tinygpsplus/
Msg type: http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html

Arduino Raw Data: http://tronixstuff.com/2010/09/17/moving-forward-with-arduino-chapter-17-gps/


When following this page (http://wiki.ros.org/nmea_navsat_driver):

** MAKE SURE YOU RUN “roscore” on a separate terminal window BEFORE YOU TRY TO RUN the cmd “rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0 _baud:=38400” 

** ALSO NOTE that your port may change i.e. /dev/ttyACM0 or /dev/ttyACM1


# Arduino Upload:
Put into dline while uploading onto Arduino
Won’t work otherwise
Put on uart and change baud rate( preferably 4800)

# Publishing to ROS:
Open 3 terminals

1st Terminal

`roscd`

`roscore`

2nd Terminal
  
`rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0 _baud:=4800`    (NOTE: PORT MAY BE /dev/ttyACM0)

3rd Terminal
 
`rostopic echo /fix`
