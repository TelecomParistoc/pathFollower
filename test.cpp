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
        {700, 100},
        {500, 100},
        {800, 100},
        {500, 100}
    };

    initMotionController();
    setRobotDistance(0);
    setRobotHeading(0);

    //setCurrentLocation(300,1110);
    setCurrentLocation(500,100);
    followPath(path, 4, 0, NULL);

    PathFollower::resetPosition(std::pair<double,double>(500,100));
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
