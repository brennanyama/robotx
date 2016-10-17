#include "maneuvers.h"

void survey(ros::Publisher pub, Polygon shape) {
    int lx, ly, ux, uy;
    int posx, posy;

    lx = shape.startX();
    ly = shape.startY();
    ux = shape.endX();
    uy = shape.endY();

    std_msgs::String msg;

    std::stringstream ss;

    ss << "GoTo(" << lx << "," << ly << ")";

    pub.publish(msg);
    ROS_INFO("[Node] mission published: %s", ss);
}

void dock(int, int) {

}

void undock(int, int) {

}

void circle(int, int, float, float, float) {

}
