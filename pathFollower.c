#include "pathFollower.h"
#include "headingcontroller.h"
#include "speedcontroller.h"
#include "point.h"
#include <math.h>


void follow(stringstream s) {
	double currentHeading = getCurrentHeading();
	float x, y;
	s >> x >> y;
	Point points[2] {Point(x, y), Point(x, y)};
	while (s >> x >> y) {
		points[0] = points[1];
		points[1] = Point(x, y);
		setTargetHeading(getAngle(points[0], points[1]);)
		queueStopAt(getDistance, NULL);	
	}	
		
}

double getAngle(Point start, Point stop) {
	float adj = abs(y2 - y1)
	float hyp = sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 -x1));
	double angle = acos(adj / hyp);
	return angle;
}

double getDistance (Point start, Point stop) {
	float dist = sqrt((stop.y - start.y)^2 + (stop.x - start.x)^2;
	return (double)dist;
}

