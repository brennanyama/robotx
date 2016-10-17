#include "status.h"
#import <string>

using namespace std;

//
// Created by Kelan on 10/16/2016.
//
class state() {
    motor = enable;
    path = enable;
    lidar = enable;
    camera = enable;
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
        default:
            break;
    }
}