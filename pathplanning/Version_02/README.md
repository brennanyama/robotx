# Dstar instructions

## How it works

When you run dstar.m, the script first subscribes to certain topics and sets up the publishers for the path x and y coordinates. The most important topic subscribed to is the /map topic, which is the dstar receives it's Occupancy Grid from ROS.

Next, the ROS OGRid must be converted to a matlab BinaryOccupancyGrid, and then to a matrix.

The Dstar algorithm is then called as follows:

* `ds = Dstar(map)  //Where map is a matrix. This initialized the algorithm object.`
* `ds.plan(goal)    //Goal is the goal point received from ROS. This part takes the longest, as it plans a route from all points to the goal`
* `spath = ds.path(start)   //start is the current location. path returns the coordinates for all the points that create the optimal path from start to goal`

Finally, the path x and y points are sent to the topics /pathX, and /pathY.
