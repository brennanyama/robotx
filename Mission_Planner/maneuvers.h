//
// Created by Kelan on 10/16/2016.
//

#ifndef MISSION_PLANNER_MANEUVERS_H
#define MISSION_PLANNER_MANEUVERS_H
void survey(ros::Publisher, int, int);
void dock(int, int);
void undock(int, int);
void circle(int, int, float, float, float);

#endif //MISSION_PLANNER_MANEUVERS_H
