#include "pathfollower.h"
#include "pathFollower.hpp"

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
