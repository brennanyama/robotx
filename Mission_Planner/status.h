//
// Created by Kelan on 10/16/2016.
//

#ifndef MISSION_PLANNER_STATUS_H
#define MISSION_PLANNER_STATUS_H

enum Status {enable, disable, error};

class state {
    Status motor;
    Status path;
    Status lidar;
    Status camera;

public:
    void change_status(System, Status);
};
#endif //MISSION_PLANNER_STATUS_H
