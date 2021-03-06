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
std::list<std::array<std::pair<double,double>,2> > PathFollower::positionAfterRecalibration;
std::list<double> PathFollower::distancesRecalibration;
std::list<int> PathFollower::type_recal;
std::pair<double,double> PathFollower::prevPosition;
std::pair<double,double> PathFollower::currentPosition;
std::pair<double,double> PathFollower::currentDirection;
double PathFollower::currentAngle = 0;
double PathFollower::radiusPositiveSpeed = 113;
double PathFollower::radiusNegativeSpeed = 40;
bool PathFollower::paused = false;
double PathFollower::remainingDistance = 0;

void PathFollower::setCurrentPosition(double x, double y) {
    std::cout<<"On remplace la position imaginee "<<curPosX<<" "<<curPosY<<" par "<<x<<" "<<y<<std::endl;
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

void PathFollower::followPath(const std::vector<double>& path_to_copy)
{
    if(path_to_copy.size()<2)
    {
        std::cout<<"WARNING : the path is empty, ignoring ..."<<std::endl;
        return;
    }

    std::vector<double> path = path_to_copy;

    // make sure everything is clean
    angles.clear();
    distances.clear();

    std::pair<double,double> angleDistance = getAngleDistance(curPosX,curPosY,path[0],path[1]);
    if(angleDistance.second>5)
    {
        angles.push_back(angleDistance.first);
        distances.push_back(angleDistance.second);
        std::cout<<"Examining "<<path[0]<<" "<<path[1]<<std::endl;
        if(isOutsideLand(path[0],path[1]))
        {
            int type;
            double dist;
            auto projected = projectInLand(path[0],path[1],curPosX,curPosY,type,dist);
            /*angleDistance = getAngleDistance(projected.first.first,projected.first.second,projected.second.first,projected.second.second);
            angles.push_back(angleDistance.first);
            distances.push_back(angleDistance.second);*/
            path[0] = projected[1].first;
            path[1] = projected[1].second;
            std::cout<<"Projeted : going to "<<path[0]<<" "<<path[1]<<" and then "<<projected[1].first<<" "<<projected[1].second<<std::endl;
            recalibrate.push_back(true);
            positionAfterRecalibration.push_back(projected);
            type_recal.push_back(type);
            distancesRecalibration.push_back(dist);
            std::cout<<"DISTANCE WHEN RECALIBRATION "<<dist<<std::endl;
        }
        else
            recalibrate.push_back(false);
    }
    else
    {
        path[0] = curPosX;
        path[1] = curPosY;
    }

    for(unsigned int i=2;i<path.size();i+=2)
    {
        std::pair<double,double> angleDistance = getAngleDistance(path[i-2],path[i-1],path[i],path[i+1]);
        std::cout<<"Examining "<<path[i]<<" "<<path[i+1]<<std::endl;
        if(angleDistance.second>5)
        {
            angles.push_back(angleDistance.first);
            distances.push_back(angleDistance.second);
            if(isOutsideLand(path[i],path[i+1]))
            {
                int type;
                double dist;
                auto projected = projectInLand(path[i],path[i+1],path[i-2],path[i-1],type,dist);
                /*angleDistance = getAngleDistance(projected.first.first,projected.first.second,projected.second.first,projected.second.second);
                angles.push_back(angleDistance.first);
                distances.push_back(angleDistance.second);*/
                //on accepte un leger decalage qui depend de si le robot va en avant ou en arriere
                path[i] = projected[1].first;
                path[i+1] = projected[1].second;
                std::cout<<"Projeted : going to "<<path[i]<<" "<<path[i+1]<<" and then "<<projected[1].first<<" "<<projected[1].second<<std::endl;
                recalibrate.push_back(true);
                positionAfterRecalibration.push_back(projected);
                type_recal.push_back(type);
                distancesRecalibration.push_back(dist);
                std::cout<<"DISTANCE WHEN RECALIBRATION "<<dist<<std::endl;
            }
            else
                recalibrate.push_back(false);
        }
        else
        {
            path[i] = path[i-2];
            path[i+1] = path[i-1];
        }
        curPosX = path[i];
        curPosY = path[i+1];
    }

    negativeSpeed = false;

    double angle = fmod(fmod(getRobotHeading(),360.0)+360.0,360.0);
    if(angle>=180.0)
        angle -= 360.0;
    double angleModulo = fmod(fabs(angles.front()-angle)+360.0,360.0);
    if(angleModulo <= 90.0 || angleModulo > 270.0)
        setTargetHeading(angles.front(), &PathFollower::standardCallback);
    else
    {
        negativeSpeed = !negativeSpeed;
        setTargetHeading(fmod(180.0+angles.front(),360.0), &PathFollower::standardCallback);
    }
    angles.pop_front();
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
        if(recalibrate.front())
            std::cout<<"Recalibrating"<<std::endl;
        std::cout<<"going of "<<distances.front()<<" "<<negativeSpeed<<" "<<cruiseSpeed<<std::endl;
        if(negativeSpeed)
        {
            queueSpeedChange(-cruiseSpeed, nullptr);
            if(recalibrate.front())
            {
                std::cout<<"moving "<<-distancesRecalibration.front()<<std::endl;
                setActiveDetectors(none);
                setRecalibrationCallback(PathFollower::whenBlockedRecalibration);
                //queueSpeedChangeAt(-distancesRecalibration.front(), -0.1, &PathFollower::disableHeading);
            }
            if(distances.size() == 1 && endSpeed != 0)
                queueSpeedChangeAt(-distances.front(), endSpeed, &PathFollower::rotateCallback);
            else
                queueStopAt(-distances.front(), &PathFollower::rotateCallback);
            remainingDistance = -distances.front();
        }
        else
        {
            queueSpeedChange(cruiseSpeed, nullptr);
            if(recalibrate.front())
            {
                std::cout<<"moving "<<distancesRecalibration.front()<<std::endl;
                setActiveDetectors(none);
                setRecalibrationCallback(PathFollower::whenBlockedRecalibration);
                //queueSpeedChangeAt(distancesRecalibration.front(), 0.1, &PathFollower::disableHeading);
            }
            if(distances.size() == 1 && endSpeed != 0)
                queueSpeedChangeAt(distances.front(), endSpeed, &PathFollower::rotateCallback);
            else
                queueStopAt(distances.front(), &PathFollower::rotateCallback);
            remainingDistance = distances.front();
        }
        if(recalibrate.front())
        {
            std::cout<<"Recalibration enclenchee "<<type_recal.front()<<std::endl;
            distancesRecalibration.pop_front();
        }
        recalibrate.pop_front();
        distances.pop_front();
    }
}

void PathFollower::rotateCallback(struct motionElement* element)
{
    if(angles.size())
    {
        std::cout<<"turning of "<<angles.front()<<" current heading : "<<getRobotHeading()<<std::endl;
        double angle = fmod(fmod(getRobotHeading(),360.0)+360.0,360.0);
        if(negativeSpeed)
            angle = 180.f+angle;
        if(angle>=180.0)
            angle -= 360.0;
        std::cout<<"Negative speed ? "<<negativeSpeed<<" "<<angle<<" and dest_angle "<<angles.front()<<" "<<angles.front()-angle<<" "<<fabs(angles.front()-angle)+360.0<<" "<<fmod(fabs(angles.front()-angle)+360.0,360.0)<<std::endl;
        double angleModulo = fmod(fabs(angles.front()-angle)+360.0,360.0);
        if(angleModulo <= 90.0 || angleModulo > 270.0)
        {
            std::cout<<"We don't inverse speed !"<<negativeSpeed<<" with angle "<<angle<<std::endl;
            if(negativeSpeed)
                setTargetHeading(fmod(180.0+angles.front(),360.0), &PathFollower::standardCallback);
            else
                setTargetHeading(angles.front(), &PathFollower::standardCallback);
        }
        else
        {
            std::cout<<"We inverse speed !"<<!negativeSpeed<<" with angle "<<angle<<std::endl;
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

std::array<std::pair<double,double>,2> PathFollower::projectInLand(int x, int y, int prevX, int prevY, int& type, double& dist)
{
    std::array<std::pair<double,double>,2> res;
    res[0].first = x;
    res[0].second = y;

    //on se recalibre en x, y reste inchangé
    if(x<0)
    {
        type = 0;
        res[0].first = radiusPositiveSpeed;
        res[0].second = y+(prevY-y)/(prevX-x)*(res[0].first-x);
        res[1].first = radiusNegativeSpeed;
        res[1].second = y+(prevY-y)/(prevX-x)*(res[1].first-x);
    }
    else if(x>3000)
    {
        type = 1;
        res[0].first = 3000-radiusPositiveSpeed;
        res[0].second = y+(prevY-y)/(prevX-x)*(res[0].first-x);
        res[1].first = 3000-radiusNegativeSpeed;
        res[1].second = y+(prevY-y)/(prevX-x)*(res[1].first-x);
    }
    else
    {
        //on se recalibre en y, x reste inchangé
        if(y<0)
        {
            type = 2;
            res[0].second = radiusPositiveSpeed;
            res[0].first = x+(prevX-x)/(prevY-y)*(res[0].second-y);
            res[1].second = radiusNegativeSpeed;
            res[1].first = x+(prevX-x)/(prevY-y)*(res[1].second-y);
        }
        else if(y>2000)
        {
            type = 3;
            res[0].first = 2000-radiusPositiveSpeed;
            res[0].first = x+(prevX-x)/(prevY-y)*(res[0].second-y);
            res[1].first = 2000-radiusNegativeSpeed;
            res[1].first = x+(prevX-x)/(prevY-y)*(res[1].second-y);
        }
        else
        {
            std::cout<<"Bad recalibration with interior point";
            res[1] = res[0];
        }
    }
    std::cout<<"====================================="<<res[0].first<<" "<<res[0].second<<" "<<res[1].first<<" "<<res[1].second<<std::endl;
    dist = sqrt((prevX-x)*(prevX-x)+(prevY-y)*(prevY-y))-100;
    std::cout<<"That gives a computation distance of "<<dist<<std::endl;
    if(dist<10)
        std::cout<<"Distance computed for deceleration in recalibration is too small : "<<dist<<std::endl;
    return res;
}

void PathFollower::resetPosition(const std::pair<double,double>& v, bool force)
{
    if(force||sqrt((v.first-prevPosition.first)*(v.first-prevPosition.first)+(v.second-prevPosition.second)*(v.second-prevPosition.second))<30.)
    {
        currentPosition = v;
        prevPosition = v;
        std::cout<<"Reset position"<<std::endl;
        PathFollower::setCurrentPosition(v.first,v.second);
        updateAngleStartingMove();
    }
    else
    {
        currentPosition = prevPosition;
        PathFollower::setCurrentPosition(prevPosition.first,prevPosition.second);
        std::cout<<"=========Reset not allowed========="<<std::endl;
    }
    updateAngleStartingMove();
}

std::pair<double,double> PathFollower::getCurrentPos()
{
    double d = getDistanceSinceMoveStart();
    //std::cout<<"Passed through "<<d<<" "<<prevPosition.first<<";"<<prevPosition.second<<std::endl;
    currentPosition.first = prevPosition.first+currentDirection.first*d;
    currentPosition.second = prevPosition.second+currentDirection.second*d;
    return currentPosition;
}

std::pair<double,double> PathFollower::getCurrentDirection()
{return currentDirection;}

bool PathFollower::isSpeedPositive()
{return !negativeSpeed;}

bool PathFollower::isPaused()
{return paused;}

void PathFollower::pause()
{
    if(!paused)
    {
        //possiblement dangereux, mais devrait fonctionner
        //remainingDistance = remainingDistance-getDistanceSinceMoveStart();
        std::cout<<"Entering pause "<<remainingDistance-getDistanceSinceMoveStart()<<std::endl;
        updatePositionEndingMove();
        paused = true;
        /*clearMotionQueue();
        queueSpeedChange(0, nullptr);*/
        forceStop(1);
    }
}

void PathFollower::continueMoving()
{
    std::cout<<"On reprend le mouovement"<<std::endl;
    if(paused)
    {
        paused = false;
        std::cout<<"Negative speed "<<negativeSpeed<<" "<<remainingDistance<<std::endl;
        forceStop(0);
        /*if(negativeSpeed)
        {
            queueSpeedChange(-cruiseSpeed, nullptr);
            if(distances.size() == 1 && endSpeed != 0)
                queueSpeedChangeAt(remainingDistance, endSpeed, &PathFollower::rotateCallback);
            else
                queueStopAt(remainingDistance, &PathFollower::rotateCallback);
        }
        else
        {
            queueSpeedChange(cruiseSpeed, nullptr);
            if(distances.size() == 1 && endSpeed != 0)
                queueSpeedChangeAt(remainingDistance, endSpeed, &PathFollower::rotateCallback);
            else
                queueStopAt(remainingDistance, &PathFollower::rotateCallback);
        }*/
    }
}

void PathFollower::whenBlockedRecalibration()
{
    if(type_recal.size())
    {
        std::cout<<"Blocked ! with current angle "<<getRobotHeading()<<" and negative speed ? "<<negativeSpeed<<" "<<type_recal.front()<<std::endl;
        auto recalibrationPosition = positionAfterRecalibration.front();
        if(!negativeSpeed)
        {
            std::cout<<"Position reseted "<<recalibrationPosition[0].first<<" "<<recalibrationPosition[0].second<<std::endl;
            setCurrentLocation(recalibrationPosition[0].first,recalibrationPosition[0].second);
            resetPosition(recalibrationPosition[0]);
        }
        else
        {
            std::cout<<"Position reseted "<<recalibrationPosition[1].first<<" "<<recalibrationPosition[1].second<<std::endl;
            setCurrentLocation(recalibrationPosition[1].first,recalibrationPosition[1].second);
            resetPosition(recalibrationPosition[1]);
        }

        clearMotionQueue();
        fastSpeedChange(0);
        int type = type_recal.front();
        switch(type)
        {
            case 0:
                if(negativeSpeed)
                    setRobotHeading(0);
                else
                    setRobotHeading(180);
                break;
            case 1:
                if(negativeSpeed)
                    setRobotHeading(180);
                else
                    setRobotHeading(0);
                break;
            case 2:
                if(negativeSpeed)
                    setRobotHeading(90);
                else
                    setRobotHeading(270);
                break;
            case 3:
                if(negativeSpeed)
                    setRobotHeading(270);
                else
                    setRobotHeading(90);
                break;
            default:
                std::cout<<"Should not happen"<<std::endl;
                break;
        }
        //dans le cas d'une vitesse negative, on reajuste la distance pour aller au prochain point
        if(distances.size()&&!negativeSpeed)
            *(distances.begin()) += radiusPositiveSpeed-radiusNegativeSpeed;
        std::cout<<"Next distance is "<<*(distances.begin())<<std::endl;
        setRobotDistance(0);
        enableHeadingControl(1);
        type_recal.pop_front();
        positionAfterRecalibration.pop_front();
        setRecalibrationCallback(NULL);
    }
    else
    {
        setRobotDistance(0);
        enableHeadingControl(1);
    }
    setActiveDetectors(all);
    setRecalibrationCallback(NULL);
    rotateCallback(NULL);
}

void PathFollower::disableHeading(motionElement*)
{
    std::cout<<"Disable heading, End of long distance, recalibration callback set"<<std::endl;
    fastSpeedChange(-0.1);
    setRecalibrationCallback(PathFollower::whenBlockedRecalibration);
    enableHeadingControl(0);
    exit(0);
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
    double d = fabs(getDistanceSinceMoveStart());
    std::cout<<"Position update "<<d<<" "<<prevPosition.first<<" "<<prevPosition.second<<std::endl;
    auto saved = currentPosition;
    currentPosition.first = prevPosition.first+currentDirection.first*d;
    currentPosition.second = prevPosition.second+currentDirection.second*d;
    prevPosition = saved;
}

void PathFollower::setRadiusPositiveSpeed(double r)
{radiusPositiveSpeed = r;}

void PathFollower::setRadiusNegativeSpeed(double r)
{radiusNegativeSpeed = r;}
