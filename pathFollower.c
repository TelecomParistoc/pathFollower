#include "pathFollower.h"
#include "headingcontroller.h"
#include "speedcontroller.h"
#include "point.h"
#include <math.h>


void follow(std::stringstream s) {
	float x, y;
	s >> x >> y;
	Point points[2] {Point(x, y), Point(x, y)};
	while (s >> x >> y) {
		points[0] = points[1];
		points[1] = Point(x, y);
		setTargetHeading(getAngle(points[0], points[1]), NULL);
		queueStopAt(getDistance(points[0], points[1]), NULL);	
	}	
		
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

