#include "status.h"
#import <string>

using namespace std;

class state() {
    motor = enable;
    path = enable;
    lidar = enable;
    camera = enable;
    event = survey;
}

class ~state() {

}

void change_status (string system, Status status) {
    switch(system) {
        case 'motor':
            motor = status;
            break;
        case 'path':
            path = status;
            break;
        case 'lidar':
            lidar = status;
            break;
        case 'camera':
            camera = status;
            break;
        case 'event':
            event = status;
            break;
        default:
            break;
    }
}

Event get_event() {
    return event;
}