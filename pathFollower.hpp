#ifndef PATHFOLLOWER_HPP
#define PATHFOLLOWER_HPP


#include <functional>
#include <iostream>
#include <vector>
#include <cmath>
#include <list>
#include <robotdriver/speedcontroller.h>
#include "pathfollower.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif


class PathFollower
{
    public:
        static void setCurrentPosition(double x, double y);
        // set only one coordinate (useful when recalibrating)
        static void setCurrentX(double value);
        static void setCurrentY(double value);
        // set the speed to robot will travel at along the path
        static void setCruiseSpeed(double speed);
        // set the robot speed at the end of the path (useful to avoid wasting time when starting actions)
        static void setEndSpeed(double speed);
        // set the callback called at the end of the path
	    static void setEndCallback(void (*callback)(void));

        // C friendly
        static void followPath(const struct robotPoint* points, const int length);
        // not C friendly. Baaaad
        static void followPath(const std::vector<double>& path);
	    static void followPath(const std::vector<double>& points, void (*endCallback)(void), double endSpeed = 0);

        static std::pair<double,double> getAngleDistance(double x1, double y1, double x2, double y2);

        // when finishing a turn
        static void standardCallback();
        // when finishing a translation
        static void rotateCallback(struct motionElement* element);

    private:
	    static bool negativeSpeed;
        static double curPosX, curPosY;
        static double cruiseSpeed, endSpeed;

        static void (*endCallback)(void);

        static std::list<double> angles, distances;
};


#endif
