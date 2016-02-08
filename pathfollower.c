#include "pathfollower.h"

void setCurrentLocation(double x, double y) {
    PathFollower::setCurrentPosition(x, y);
}
void setCurrentX(double x) {
    PathFollower::setCurrentX(x);
}
void setCurrentY(double y) {
    PathFollower::setCurrentY(y);
}

void followPath(struct robotPoint* points, int size, double endSpeed, void (*endCallback)(void)) {
    PathFollower::followPath(points, size);
}
