/* This file displays basic usage of the pathfollower lib.
 *
 * developped by J. "JS" Schoumacher and A. Bonetti for Telecom Robotics */

#include <pathfollower/pathfollower.h>
#include <robotdriver/speedcontroller.h>
#include <robotdriver/motioncontroller.h>
#include <robotdriver/motordriver.h>
#include <stdio.h>

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

int main() {
    // the path the robot has to follow ({x, y} in mm)
    /*struct robotPoint path[] = {
        {330, 280},
        {360, 250},
        {390, 240},
        {430, 230},
        {480, 220},
        {480, 210},
        {490, 210}
    };*/

    struct robotPoint path[] = {
        {800, 100},
        {1100, 100}
    };

    initMotionController();
    setRobotDistance(0);
    setRobotHeading(0);

    //setCurrentLocation(300,1110);
    setCurrentLocation(500,100);
    followPath(path, 2, 0, NULL);

    while(1);

    return 0;
}
