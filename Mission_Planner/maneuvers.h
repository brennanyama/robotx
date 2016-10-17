#ifndef MISSION_PLANNER_MANEUVERS_H
#define MISSION_PLANNER_MANEUVERS_H
#include "shape.h"

void survey(ros::Publisher, Polygon);
void dock(int, int);
void undock(int, int);
void circle(int, int, float, float, float);

#endif //MISSION_PLANNER_MANEUVERS_H
