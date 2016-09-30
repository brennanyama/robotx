Updated by: Paulo Lemus
Date: 9/30/16

Update 9/30/16:

1. Added ROS subscriber/publisher lines to Dstar. The subscriber receives the most
recent message type of 'nav_msgs/OccupancyGrid'. This may need to be converted to a matrix so that is is compatible with 
Dstar.

2. Added convG2M function, meant to convert the Occupancy of a OccupancyGrid to a 
Matrix.

TODO:
Add subscribers for positional GPS data for current position. 
Test GPS input with GPS for pos[latn, longn]
Add function to normalize or represent data as an int.

