/* This file displays basic usage of the pathfollower lib.
 *
 * developped by J. "JS" Schoumacher and A. Bonetti for Telecom Robotics */

#include <pathfollower/pathfollower.h>
#include <robotdriver/motioncontroller.h>
#include <stdio.h>

// called when the robot has reached the end of the path
void onTheEndOfTheRoad() {
    printf("I've travelled a long way, and now I reached the end of my path.\n");
}

int main() {
    // the path the robot has to follow ({x, y} in mm)
    struct robotPoint path[] = {
        {700, 70},
        {1200, 900},
        {200, 200},
        {900, 1850}
    };

    initMotionController();
    setRobotDistance(0);

    setCurrentLocation(200,900);
    followPath(path, 4, 0, onTheEndOfTheRoad);

    while(1);

    return 0;
}
