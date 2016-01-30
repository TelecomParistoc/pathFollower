#include "pathFollower.hpp"


float PathFollower::curPosX = 0;
float PathFollower::curPosY = 0;
float PathFollower::curAngle = 0;

std::list<float> PathFollower::angles;


void PathFollower::setCurrentPositionDirection(float x, float y, float dirX, float dirY)
{
    curPosX = x;
    curPosY = y;

    double dst = sqrt(dirX*dirX+dirY*dirY);
    double acos1 = acos(dirX/dst);
    double asin1 = asin(dirY/dst);

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
}

void PathFollower::followPath(const std::vector<float>& path)
{
    setRobotDistance(0);

    if(path.size()<2)
        return;

    std::pair<float,float> angleDistance = getAngleDistance(curPosX,curPosY,path[0],path[1]);
    turnOf(angleDistance.first, &PathFollower::standardCallback);
    curAngle = 0; //after beeing set, the currrent angle is in getRobotHeading

    queueSpeedChange(0.3, nullptr);
    queueStopAt(angleDistance.second, std::bind<void(struct motionElement* element)>(&PathFollower::rotateCallback,this,std::placeholders::_2));

    for(unsigned int i=2;i<path.size();i+=2)
    {
        std::pair<float,float> angleDistance = getAngleDistance(path[i-2],path[i-1],path[i],path[i+1]);
        angles.push_back(angleDistance.first);
        curPosX = path[i];
        curPosY = path[i+1];

        queueSpeedChange(0.3, nullptr);
		queueStopAt(angleDistance.second, std::bind<void(struct motionElement* element)>(&PathFollower::rotateCallback,this,std::placeholders::_2));
    }

    queueSpeedChange(0.3, nullptr);
	queueStopAt(angleDistance.second, &PathFollower::endCallback);
}

std::pair<float,float> PathFollower::getAngleDistance(float x1, float y1, float x2, float y2)
{
    std::pair<float,float> ret;

    ret.second = sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));

    //python -c 'import math;x1=200;y1=900;x2=700;y2=50;d=math.sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));print(math.acos((x2-x1)/d));print(math.asin((y2-y1)/d))'
    double acos1 = acos((x2-x1)/ret.second);
    double asin1 = asin((y2-y1)/ret.second);

    float angle1;

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

    ret.first = angle1-getRobotHeading()-curAngle;

	return ret;
}

void PathFollower::standardCallback(void*)
{
	setRobotDistance(0);
}

void PathFollower::rotateCallback(PathFollower* p, struct motionElement* element)
{
	turnOf(p->angles.front(), standardCallback);
	p->angles.pop_front();
}

void PathFollower::endCallback(struct motionElement* element)
{}
