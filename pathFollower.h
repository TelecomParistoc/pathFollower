#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include "point.h"

void follow(float path[], int pathLength);
double getAngle(Point, Point);
double getDistance(Point, Point);

void standardCallback(struct motionElement* element);
void rotateCallback(struct motionElement* element);
void endCallback(struct motionElement* element);

#endif
