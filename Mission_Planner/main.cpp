#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "status.h"
#include "maneuvers.h"
#include "events.h"
#include "shape.h"


int main(int argc, char **argv) {
    state WAMV = new state();

    ros::init(argc, argv, "mission_planner");

    ros::NodeHandle nh;

    // Publishers
    ros::Publisher mission_pub = nh.advertise<std_msgs::String>("mission", 1000);   // 'mission' publisher

    // Subscribers
    ros::Subscriber motor_sub = nh.subscribe("motor_status", 1000, WAMV.change_status); // 'motor_status' subscriber
    ros::Subscriber sensor_sub = nh.subscribe("sensor_status", 1000, eventAlert);       // 'sensor_status' subscriber
    ros::Subscriber manual_sub = nh.subscribe("manual_status", 1000, WAMV.change_status);   // 'manual_status' subscriber
    ros::Subscriber kill_sub = nh.subscribe("kill_status", 1000, WAMV.change_status);       // 'kill_status' subscriber

    ros::Rate loop_rate(10);        // Loop at 10 Hz


    int count = 0;

    while (ros::ok()) { // This is the main loop
        Event ev = WAMV.get_event();

        if(ev == survey) {survey(mission_pub, polygon);}                   // Begin Surveying the area
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}