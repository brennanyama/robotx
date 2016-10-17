#ifndef MISSION_PLANNER_SHAPE_H
#define MISSION_PLANNER_SHAPE_H
class Polygon {
    int start_x;
    int start_y;
    int end_x;
    int end_y;

public:
    int startX();
    int startY();
    int endX();
    int endY();
};

#endif //MISSION_PLANNER_SHAPE_H
