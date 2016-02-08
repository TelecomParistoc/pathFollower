#ifndef PATHFOLLOWER_HPP
#define PATHFOLLOWER_HPP


#include <functional>
#include <iostream>
#include <vector>
#include <cmath>
#include <list>

#include "robotdriver/headingcontroller.h"
#include "robotdriver/motioncontroller.h"
#include "robotdriver/speedcontroller.h"
#include "robotdriver/motordriver.h"
#include "pathfollower.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif


class PathFollower
{
    public:
        static void setCurrentPosition(double x, double y);
        static void setCurrentPositionDirection(double x, double y, double dirX, double dirY);
        static void setCurrentX(double value);
        static void setCurrentY(double value);
        
        static void followPath(const struct robotPoint* points, const int length);
        static void followPath(const std::vector<double>& path);

        static std::pair<double,double> getAngleDistance(double x1, double y1, double x2, double y2);

        static void standardCallback();
        static void rotateCallback(struct motionElement* element);

    private:
        static double curPosX, curPosY;
        static double curAngle;

        static std::list<double> angles, distances;
};


#endif
