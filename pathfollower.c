#include "pathfollower.h"
#include "pathFollower.hpp"
#include <librobot/robot.h>

void setCurrentLocation(double x, double y) {
	PathFollower::setCurrentPosition(x, y);
	PathFollower::resetPosition(std::pair<double, double>(x, y));
}

void setCruiseSpeed(double speed) {
    PathFollower::setCruiseSpeed(speed);
}

void followPath(struct robotPoint* points, int size, double endSpeed, void (*endCallback)(void)) {
    PathFollower::followPath(points, size);
    PathFollower::setEndSpeed(endSpeed);
    PathFollower::setEndCallback(endCallback);
}

void ffollow(const char * pathName, void (*endCallback)(void)) {
    std::string pathfile(pathName);
    int config = getTableConfig();
    if(config == 0)
        config = 1;
    if(getTeam() == GREEN_TEAM) {
        pathfile = "/var/paths/" + pathfile + "-green-" + std::to_string(config)+".path";
    } else {
        pathfile = "/var/paths/" + pathfile + "-purple-" + std::to_string(config)+".path";
    }
    std::cout << pathfile << std::endl;
    PathFollower::followPath(pathfile);
    PathFollower::setEndCallback(endCallback);
}

double getCurrentX() {
	return(PathFollower::getCurrentPos().first);
}

double getCurrentY() {
	return(PathFollower::getCurrentPos().second);
}

void updateAngleStartingMove() {
	PathFollower::updateAngleStartingMove();
}

void updatePositionEndingMove() {
	PathFollower::updatePositionEndingMove();
}
