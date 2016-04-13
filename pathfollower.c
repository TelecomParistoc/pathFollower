#include "pathfollower.h"
#include "pathFollower.hpp"
#include <librobot/robot.h>

void setCurrentLocation(double x, double y) {
    PathFollower::setCurrentPosition(x, y);
}
void setCurrentX(double x) {
    PathFollower::setCurrentX(x);
}
void setCurrentY(double y) {
    PathFollower::setCurrentY(y);
}

void setCruiseSpeed(double speed) {
    PathFollower::setCruiseSpeed(speed);
}

void followPath(struct robotPoint* points, int size, double endSpeed, void (*endCallback)(void)) {
    PathFollower::followPath(points, size);
    PathFollower::setEndSpeed(endSpeed);
    PathFollower::setEndCallback(endCallback);
}

void ffollow(char * pathName, void (*endCallback)(void)) {
    std::string pathfile(pathName);
    int config = getTableConfig();
    if(config == 0)
        config = 1;
    if(getTeam() == GREEN_TEAM) {
        pathfile = "/var/paths/" + pathfile + "-green-" + std::to_string(config);
    } else {
        pathfile = "/var/paths/" + pathfile + "-purple-" + std::to_string(config);
    }
    PathFollower::followPath(pathfile);
    PathFollower::setEndCallback(endCallback);
}
