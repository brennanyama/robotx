#include <string>
using namespace std;

#ifndef MISSION_PLANNER_STATUS_H
#define MISSION_PLANNER_STATUS_H

enum Status {enable, disable, error};
enum Event {survey, obstacle, manual, kill};

class state {
    Status motor;
    Status path;
    Status lidar;
    Status camera;
    Event event;

public:
    void change_status(string, Status);
    Event get_event();
};
#endif //MISSION_PLANNER_STATUS_H
