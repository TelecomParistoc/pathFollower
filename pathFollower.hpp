#ifndef PATHFOLLOWER_HPP
#define PATHFOLLOWER_HPP


#include <functional>
#include <iostream>
#include <fstream>
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
        // not C friendly. Baaaad. TG arnaud
        /**
        First follow path take a filename as argument, which must be parsed as follow :
        endSpeed [space or carriage return] cruiseSpeed [space or carriage return] (x [space or carriage return] y [space or carriage return])*N
        where N could be arbitrary long or not
        **/
        static void followPath(const std::string& pathOfPath);
        static void followPath(const std::vector<double>& path);
	    static void followPathCallback(const std::vector<double>& points, void (*endCallback)(void), double endSpeed = 0);

        static std::pair<double,double> getAngleDistance(double x1, double y1, double x2, double y2);

        // when finishing a turn
        static void standardCallback();
        // when finishing a translation
        static void rotateCallback(struct motionElement* element);

        static bool isOutsideLand(int x, int y);
        static std::pair<std::pair<double,double>,std::pair<double,double> > projectInLand(int x, int y, int prevX, int prevY, int& type, float& dist);

        static void resetPosition(const std::pair<double,double>& v);
        static std::pair<double,double> getCurrentPos();
        static std::pair<double,double> getCurrentDirection();

        static void whenBlockedRecalibration();
        static void disableHeading(motionElement* m);
        static void updateAngleStartingMove();
        static void updatePositionEndingMove();

        static void setRadius(double r);
        static void setDistanceToGoAway(double d);

    private:
	    static bool negativeSpeed;
        static double curPosX, curPosY;
        static double cruiseSpeed, endSpeed;

        static void (*endCallback)(void);

        static std::list<double> angles, distances;
        static std::list<bool> recalibrate;
        static std::list<std::pair<double,double> > positionAfterRecalibration;
        static std::list<float> distancesRecalibration;
        static std::list<int> type_recal;

        static std::pair<double,double> prevPosition;
        static std::pair<double,double> currentPosition;
        static std::pair<double,double> currentDirection;
        static double currentAngle;

        static double radius;
        static double distanceToGoAway;
};


#endif
