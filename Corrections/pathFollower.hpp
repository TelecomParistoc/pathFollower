#ifndef PATHFOLLOWER_HPP
#define PATHFOLLOWER_HPP


#include <vector>
#include <cmath>
#include <list>

#include "headingcontroller.h"
#include "speedcontroller.h"


#ifndef
#define M_PI 3.14159265
#endif


class PathFollower
{
    public:
        static void setCurrentPositionDirection(float x, float y, float dirX, float dirY);
        static void followPath(const std::vector<float>& path);

        static std::pair<float,float> getAngleDistance(float x1, float y1, float x2, float y2);

        void standardCallback(struct motionElement* element);
        void rotateCallback(struct motionElement* element);
        void endCallback(struct motionElement* element);

    private:
        static float curPosX, curPosY;
        static float curAngle;

        static std::list<float> angles;
};


#endif
