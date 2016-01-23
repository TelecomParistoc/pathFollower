#include "pathFollower.h"
#include "headingcontroller.h"
#include "speedcontroller.h"
#include <math.h>

double angle;
int colour;

void follow(float path[], int pathLength) {
	Point points[3];

	setRobotDistance(0);

	points[0].x = path[0];
	points[0].y = path[1];
	points[1].x = path[0];
	points[1].y = path[1];
	points[2].x = path[2];
	points[2].x = path[3];

	turnOf(getAngle(points[0], points[2]), standardCallback);
	double robotHeading;
	for (int i = 2; i < pathLength - 1; i++) {
		points[0] = points[1];
		points[1] = points[2];
		points[2].x = path[2 * i];
		points[2].y = path[2 * i + 1];
		if (colour) {
			robotHeading = getRobotHeading() + 90;
		} else {
			robotHeading = getRobotHeading() - 90;
		}
		angle = robotHeading - getAngle(points[1], points[2]);
		queueSpeedChange(0.3, NULL);
		queueStopAt(getDistance(points[0], points[1]), rotateCallback);	
	}
	queueSpeedChange(0.3, NULL);
	queueStopAt(getdDistance(points[1], points[2]), endCallback);
}

double getAngle(Point start, Point stop) {
	float adj = abs(stop.y - start.y);
	float hyp = sqrt((stop.y - start.y) * (stop.y - start.y)  + (stop.x - start.x) * (stop.x - start.x));
	double angle = acos(adj / hyp);
	return angle;
}

double getDistance (Point start, Point stop) {
	float dist = sqrt((stop.y - start.y) * (stop.y - start.y) + (stop.x - start.x) * (stop.x - start.x));
	return (double)dist;
}


void standardCallback(struct motionElement* element) {
	setRobotDistance(0);
}

void rotateCallback(struct motionElement* element) {
	turnOf(angle, standardCallback);
}

void endCallback(struct motionElement* element) {
}
