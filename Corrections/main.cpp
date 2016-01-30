#include <iostream>

#include "pathFollower.hpp"


int main()
{
    initMotionController();
    setRobotDistance(0);

    std::vector<float> pointsToVisit;
    pointsToVisit.push_back(200);
    pointsToVisit.push_back(900);
    pointsToVisit.push_back(700);
    pointsToVisit.push_back(50);
    pointsToVisit.push_back(1200);
    pointsToVisit.push_back(900);
    pointsToVisit.push_back(100);
    pointsToVisit.push_back(200);
    pointsToVisit.push_back(900);
    pointsToVisit.push_back(1850);

    PathFollower::setCurrentPositionDirection(200,900,1,0);
    PathFollower::followPath(pointsToVisit);

    while(1);

    return 0;
}
