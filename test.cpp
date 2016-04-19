/* This file displays basic usage of the pathfollower lib.
 *
 * developped by J. "JS" Schoumacher and A. Bonetti for Telecom Robotics */

#include <pathfollower/pathFollower.hpp>
#include <robotdriver/speedcontroller.h>
#include <robotdriver/motioncontroller.h>
#include <robotdriver/toolboxdriver.h>
#include <robotdriver/motordriver.h>
#include <librobot/robot.h>
#include <iostream>


int main()
{
    initRobot();
    setRGB(255, 0, 0);


    setMoveStartCallback(&PathFollower::updateAngleStartingMove);
    setMoveEndCallback(&PathFollower::updatePositionEndingMove);
    setCurrentLocation(40,1000);
    ffollow("bidon", NULL);

    PathFollower::resetPosition(std::pair<double,double>(40,1000));

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
