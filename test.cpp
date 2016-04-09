/* This file displays basic usage of the pathfollower lib.
 *
 * developped by J. "JS" Schoumacher and A. Bonetti for Telecom Robotics */

#include <pathfollower/pathFollower.hpp>
#include <robotdriver/motioncontroller.h>
#include <robotdriver/speedcontroller.h>
#include <robotdriver/toolboxdriver.h>
#include <robotdriver/motordriver.h>
#include <unistd.h>
#include <iostream>

/*
// called when the robot has reached the end of the path
void onTheEndOfTheRoad() {
    struct robotPoint path[] = {
        {480, 180},
        {180, 1160}
    };
    printf("I've travelled a long way, and now I reached the end of my path.\n");
    //followPath(path, 2, 0, NULL);
}*/

int main()
{
    struct robotPoint path[] = {
        {800, 1000},
        {800, -200},
        {800, 1000},
        {500, 100}
    };
    initMotionController();
    setRobotDistance(0);
    setRobotHeading(0);

    setCurrentLocation(800,1000);
    followPath(path, 4, 0, NULL);

    PathFollower::resetPosition(std::pair<double,double>(800,1000));
    //TODO: mesurer le rayon du robot quand il avance => PathFollower::setRadius();
    //TODO: mesurer la distance de recul optimale => PathFollower::setDistanceToGoAway();
    setMoveStartCallback(&PathFollower::updateAngleStartingMove);
    setMoveEndCallback(&PathFollower::updatePositionEndingMove);

    std::pair<double,double> curPos;
    std::pair<double,double> curDir;
    while(1)
    {
        curPos = PathFollower::getCurrentPos();
        curDir = PathFollower::getCurrentDirection();
        std::cout<<curPos.first<<" "<<curPos.second<<";"<<curDir.first<<" "<<curDir.second<<std::endl;
        waitFor(100);
    }

    return 0;
}
