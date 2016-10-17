#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "status.h"
#include "maneuvers.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_planner");

    ros::NodeHandle nh;

    ros::Publisher mission_pub = nh.advertise<std_msgs::String>("mission", 1000);   // Setup this node as a publisher named 'mission'

    ros::Rate loop_rate(10);        // Loop at 10 Hz

    int count = 0;
    while (ros::ok()) { // This is the main loop
        survey();                   // Begin Surveying the area
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}