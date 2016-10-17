#include "ros/ros.h"
#include "std_msgs/String.h"
#include "events.h"

void obstacle(int, int) {

}

void manual() {

}

void killSwitch() {

}

void eventAlert(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Node] events subscribed: %s", msg->data.c_str());
}