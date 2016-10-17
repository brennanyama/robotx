#ifndef MISSION_PLANNER_EVENTS_H
#define MISSION_PLANNER_EVENTS_H

void obstable(int, int);
void manual();
void killSwitch();
void eventAlert(const std_msgs::String::ConstPtr&);

#endif //MISSION_PLANNER_EVENTS_H
