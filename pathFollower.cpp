#include "pathFollower.hpp"
#include <robotdriver/motordriver.h>
#include <robotdriver/headingcontroller.h>
#include <robotdriver/motioncontroller.h>

double PathFollower::curPosX = 0;
double PathFollower::curPosY = 0;
double PathFollower::cruiseSpeed = 0.3;
double PathFollower::endSpeed = 0;
void (*PathFollower::endCallback)(void) = nullptr;
std::list<double> PathFollower::angles;
std::list<double> PathFollower::distances;

void PathFollower::setCurrentPosition(double x, double y) {
    curPosX = x;
    curPosY = y;
}
void PathFollower::setCurrentX(double value) {
    curPosX = value;
}
void PathFollower::setCurrentY(double value) {
    curPosY = value;
}
void PathFollower::setCruiseSpeed(double speed) {
    cruiseSpeed = speed;
}
void PathFollower::setEndSpeed(double speed) {
    endSpeed = speed;
}
void PathFollower::setEndCallback(void (*callback)(void)) {
    endCallback = callback;
}

/*void PathFollower::setCurrentPositionDirection(double x, double y, double dirX, double dirY)
{
    curPosX = x;
    curPosY = y;

    //LE BORDEL

    double dst = sqrt(dirX*dirX+dirY*dirY);
    double acos1 = acos(dirX/dst);
    double asin1 = asin(dirY/dst);
    double curAngle = 0;

    if(fabs(fabs(acos1)-fabs(asin1))<=0.000000001)
        if(asin1>=0)
            curAngle = acos1*180.f/M_PI;
        else
            curAngle = asin1*180.f/M_PI;
    else
        if(asin1>=0)
            curAngle = acos1*180.f/M_PI;
        else
            curAngle = -acos1*180.f/M_PI;
}*/

void PathFollower::followPath(const struct robotPoint* points, const int length) {
    std::vector<double> pointsToVisit;

    for(int i=0; i<length; i++) {
        pointsToVisit.push_back(points[i].x);
        pointsToVisit.push_back(points[i].y);
    }
    followPath(pointsToVisit);
}

void PathFollower::followPath(const std::vector<double>& path)
{
    setRobotDistance(0);

    if(path.size()<2) {
        std::cout<<"WARNING : the path is empty, ignoring ..."<<std::endl;
        return;
    }

    // make sure everything is clean
    angles.clear();
    distances.clear();

    std::pair<double,double> angleDistance = getAngleDistance(curPosX,curPosY,path[0],path[1]);
    angles.push_back(angleDistance.first);
    distances.push_back(angleDistance.second);

    for(unsigned int i=2;i<path.size();i+=2)
    {
        std::pair<double,double> angleDistance = getAngleDistance(path[i-2],path[i-1],path[i],path[i+1]);
        angles.push_back(angleDistance.first);
        distances.push_back(angleDistance.second);
        curPosX = path[i];
        curPosY = path[i+1];
    }

    setTargetHeading(angles.front(), &PathFollower::standardCallback);
    //std::cout<<"turning of "<<angles.front()<<std::endl;
    angles.pop_front();
}

std::pair<double,double> PathFollower::getAngleDistance(double x1, double y1, double x2, double y2)
{
    std::pair<double,double> ret;

    ret.second = sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));

    //python -c 'import math;x1=200;y1=900;x2=700;y2=50;d=math.sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));print(math.acos((x2-x1)/d));print(math.asin((y2-y1)/d))'
    double acos1 = acos((x2-x1)/ret.second);
    double asin1 = asin((y2-y1)/ret.second);

    double angle1;

    if(fabs(fabs(acos1)-fabs(asin1))<=0.000000001)
        if(asin1>=0)
            angle1 = acos1*180.f/M_PI;
        else
            angle1 = asin1*180.f/M_PI;
    else
        if(asin1>=0)
            angle1 = acos1*180.f/M_PI;
        else
            angle1 = -acos1*180.f/M_PI;

    std::cout<<x1<<" "<<y1<<" "<<x2<<" " <<y2<<" => "<<ret.second<<" avec  "<<acos1<<" "<<asin1<<" giving "<<angle1<<std::endl;

    ret.first = angle1;

	return ret;
}

void PathFollower::standardCallback()
{
	setRobotDistance(0);
    if(distances.size()) {
        // std::cout<<"going of "<<distances.front()<<std::endl;
        queueSpeedChange(cruiseSpeed, nullptr);
        if(distances.size() == 1 && endSpeed != 0)
            queueSpeedChangeAt(distances.front(), endSpeed, &PathFollower::rotateCallback);
        else
            queueStopAt(distances.front(), &PathFollower::rotateCallback);
        distances.pop_front();
    }
}

void PathFollower::rotateCallback(struct motionElement* element)
{
    if(angles.size()) {
        //std::cout<<"turning of "<<angles.front()<<std::endl;
        setTargetHeading(angles.front(), &PathFollower::standardCallback);
        angles.pop_front();
    } else { // on the end of the path
        if(endCallback != nullptr)
            endCallback();
    }
    //a supprimer
    element++;
}
