/* This file displays basic usage of the pathfollower lib.
 *
 * developped by J. "JS" Schoumacher and A. Bonetti for Telecom Robotics */

#include <pathfollower/pathfollower.h>
#include <robotdriver/speedcontroller.h>
#include <robotdriver/motioncontroller.h>
#include <robotdriver/motordriver.h>
#include <robotdriver/toolboxdriver.h>
#include <stdio.h>


// called when the robot has reached the end of the path
void onTheEndOfTheRoad() {
    printf("I've travelled a long way, and now I reached the end of my path.\n");
}

int main() {
    initMotionController();
    initToolboxDriver();
    setRobotDistance(0);
    setRobotHeading(0);

    setCurrentLocation(40,1000);
    ffollow("mon chem", onTheEndOfTheRoad);

    while(1);

    return 0;
}
