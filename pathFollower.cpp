#include "pathFollower.hpp"
#include <robotdriver/motordriver.h>
#include <robotdriver/headingcontroller.h>
#include <robotdriver/speedcontroller.h>
#include <robotdriver/motioncontroller.h>

double PathFollower::curPosX = 0;
double PathFollower::curPosY = 0;
double PathFollower::cruiseSpeed = 0.3;
double PathFollower::endSpeed = 0;
bool PathFollower::negativeSpeed = false;
void (*PathFollower::endCallback)(void) = nullptr;
std::list<double> PathFollower::angles;
std::list<double> PathFollower::distances;
std::list<bool> PathFollower::recalibrate;
std::list<std::pair<double,double> > PathFollower::positionAfterRecalibration;
std::pair<double,double> PathFollower::prevPosition;
std::pair<double,double> PathFollower::currentPosition;
std::pair<double,double> PathFollower::currentDirection;
double PathFollower::currentAngle = 0;
double PathFollower::radius = 80;
double PathFollower::distanceToGoAway = 180;

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

void PathFollower::followPath(const struct robotPoint* points, const int length) {
    std::vector<double> pointsToVisit;

    for(int i=0; i<length; i++) {
        pointsToVisit.push_back(points[i].x);
        pointsToVisit.push_back(points[i].y);
    }
    followPath(pointsToVisit);
}

void PathFollower::followPath(const std::string& pathOfPaths)
{
    std::vector<double> path;
    std::ifstream ifs(pathOfPaths.c_str(),std::ios::in);
    if(!ifs)
    {
        std::cout<<"Warning, non existing path file, ignoring ..."<<std::endl;
        return;
    }
    if(!(ifs>>endSpeed>>cruiseSpeed))
    {
        std::cout<<"Warning, non set endSpeed or cruiseSpeed, ignoring ..."<<std::endl;
        return;
    }
    double a, b;
    while(ifs>>a>>b)
    {
        path.push_back(a);
        path.push_back(b);
    }
    followPath(path);
}

void PathFollower::followPath(const std::vector<double>& path)
{
    if(path.size()<2)
    {
        std::cout<<"WARNING : the path is empty, ignoring ..."<<std::endl;
        return;
    }

    // make sure everything is clean
    angles.clear();
    distances.clear();

    std::pair<double,double> angleDistance = getAngleDistance(curPosX,curPosY,path[0],path[1]);
    if(angleDistance.second>0.1)
    {
        angles.push_back(angleDistance.first);
        distances.push_back(angleDistance.second);
        if(isOutsideLand(path[0],path[1]))
        {
            auto projected = projectInLand(path[0],path[1]);
            angleDistance = getAngleDistance(projected.first.first,projected.first.second,projected.second.first,projected.second.second);
            angles.push_back(angleDistance.first);
            distances.push_back(angleDistance.second);
            recalibrate.push_back(true);
            positionAfterRecalibration.push_back(std::pair<double,double>(projected.first.first,projected.first.second));
        }
        else
            recalibrate.push_back(false);
    }

    for(unsigned int i=2;i<path.size();i+=2)
    {
        std::pair<double,double> angleDistance = getAngleDistance(path[i-2],path[i-1],path[i],path[i+1]);
        if(angleDistance.second>0.1)
        {
            angles.push_back(angleDistance.first);
            distances.push_back(angleDistance.second);
            if(isOutsideLand(path[i],path[i+1]))
            {
                auto projected = projectInLand(path[0],path[1]);
                angleDistance = getAngleDistance(projected.first.first,projected.first.second,projected.second.first,projected.second.second);
                angles.push_back(angleDistance.first);
                distances.push_back(angleDistance.second);
                recalibrate.push_back(true);
                positionAfterRecalibration.push_back(std::pair<double,double>(projected.first.first,projected.first.second));
            }
            else
                recalibrate.push_back(false);
        }
        curPosX = path[i];
        curPosY = path[i+1];
    }

    float angle = fmod(fmod(getRobotHeading(),360.0)+360.0,360.0);
    if(angle>=180.0)
        angle -= 360.0;

    //std::cout<<"Negative speed ? "<<negativeSpeed<<std::endl;
    if(fabs(angles.front()-angle) <= 90.0)
        setTargetHeading(angles.front(), &PathFollower::standardCallback);
    else
    {
        //std::cout<<"Inverse speed => "<<!negativeSpeed<<" "<<fmod(180.0+angles.front(),360.0)<<std::endl;
        negativeSpeed = !negativeSpeed;
        setTargetHeading(fmod(180.0+angles.front(),360.0), &PathFollower::standardCallback);
    }
    angles.pop_front();
    //std::cout<<"turning of "<<angles.front()<<std::endl;
}

void PathFollower::followPathCallback(const std::vector<double>& points, void (*endCallback)(void), double endSpeed)
{
    followPath(points);
    setEndSpeed(endSpeed);
    setEndCallback(endCallback);
}

std::pair<double,double> PathFollower::getAngleDistance(double x1, double y1, double x2, double y2)
{
    std::pair<double,double> ret;

    ret.first = 0;
    ret.second = sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));

    if(ret.second<0.0000001)
        return ret;

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
    std::cout<<"Reseting distance"<<std::endl;
    if(distances.size())
    {
        //std::cout<<"going of "<<distances.front()<<" "<<negativeSpeed<<" "<<cruiseSpeed<<std::endl;
        if(negativeSpeed)
        {
            queueSpeedChange(-cruiseSpeed, nullptr);
            if(distances.size() == 1 && endSpeed != 0)
                queueSpeedChangeAt(-distances.front(), endSpeed, &PathFollower::rotateCallback);
            else
                queueStopAt(-distances.front(), &PathFollower::rotateCallback);
        }
        else
        {
            queueSpeedChange(cruiseSpeed, nullptr);
            if(distances.size() == 1 && endSpeed != 0)
                queueSpeedChangeAt(distances.front(), endSpeed, &PathFollower::rotateCallback);
            else
                queueStopAt(distances.front(), &PathFollower::rotateCallback);
        }
        if(recalibrate.front())
            setRecalibrationCallback(PathFollower::whenBlockedRecalibration);
        recalibrate.pop_front();
        distances.pop_front();
    }
}

void PathFollower::rotateCallback(struct motionElement* element)
{
    if(angles.size()) {
        //std::cout<<"turning of "<<angles.front()<<" current heading : "<<getRobotHeading()<<std::endl;
        float angle = fmod(fmod(getRobotHeading(),360.0)+360.0,360.0);
        if(angle>=180.0)
            angle -= 360.0;
        //std::cout<<"Negative speed ? "<<negativeSpeed<<" "<<angle<<" and dest_angle "<<angles.front()<<std::endl;
        if(negativeSpeed)
            angle = 180.f-angle;
        if(fabs(angles.front()-angle) <= 90.0)
            setTargetHeading(angles.front(), &PathFollower::standardCallback);
        else
        {
            //std::cout<<"We inverse speed !"<<!negativeSpeed<<std::endl;
            negativeSpeed = !negativeSpeed;
            if(negativeSpeed)
                setTargetHeading(fmod(180.0+angles.front(),360.0), &PathFollower::standardCallback);
            else
                setTargetHeading(angles.front(), &PathFollower::standardCallback);
        }
        angles.pop_front();
    } else
    { // on the end of the path
        if(endCallback != nullptr)
            endCallback();
    }
    element++;
}

bool PathFollower::isOutsideLand(int x, int y)
{
    return x<0||y<0||x>3000||y>2000;
}

std::pair<std::pair<double,double>,std::pair<double,double> > PathFollower::projectInLand(int x, int y)
{
    std::pair<std::pair<double,double>,std::pair<double,double> > res;
    res.first.first = x;
    res.first.second = y;
    //on se recalibre en x, y reste inchangé
    if(x<0)
    {
        res.first.first = radius;
        res.second.first = radius+distanceToGoAway;
    }
    else if(x>3000)
    {
        res.first.first = 3000-radius;
        res.second.first = 3000-radius-distanceToGoAway;
    }
    //on se recalibre en y, x reste inchangé
    else if(y<0)
    {
        res.first.second = radius;
        res.second.second = radius+distanceToGoAway;
    }
    else if(y>2000)
    {
        res.first.second = 2000-radius;
        res.second.second = 2000-radius-distanceToGoAway;
    }
    return res;
}

void PathFollower::resetPosition(const std::pair<double,double>& v)
{
    currentPosition = v;
    prevPosition = v;
    updateAngleStartingMove();
}

std::pair<double,double> PathFollower::getCurrentPos()
{
    double d = getDistanceSinceMoveStart();
    std::cout<<"Passed through "<<d<<std::endl;
    currentPosition.first = prevPosition.first+currentDirection.first*d;
    currentPosition.second = prevPosition.second+currentDirection.second*d;
    return currentPosition;
}

std::pair<double,double> PathFollower::getCurrentDirection()
{return currentDirection;}

void PathFollower::whenBlockedRecalibration()
{
    std::pair<double,double> recalibrationPosition = positionAfterRecalibration.front();
    setCurrentLocation(recalibrationPosition.first,recalibrationPosition.second);
    resetPosition(recalibrationPosition);
    /**TODO : cancel current move **/
    rotateCallback();
}

void PathFollower::updateAngleStartingMove()
{
    std::cout<<"Position starting "<<getRobotHeading()<<" "<<getDistanceSinceMoveStart()<<std::endl;
    currentAngle = getRobotHeading();
    currentDirection.first = cos(currentAngle/180.0*M_PI);
    currentDirection.second = sin(currentAngle/180.0*M_PI);
}

void PathFollower::updatePositionEndingMove()
{
    double d = getDistanceSinceMoveStart();
    std::cout<<"Position update "<<d<<" "<<prevPosition.first<<" "<<prevPosition.second<<std::endl;
    auto saved = currentPosition;
    currentPosition.first = prevPosition.first+currentDirection.first*d;
    currentPosition.second = prevPosition.second+currentDirection.second*d;
    prevPosition = saved;
}

void PathFollower::setRadius(double r)
{radius = r;}

void PathFollower::setDistanceToGoAway(double d)
{distanceToGoAway = d;}
