#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#ifdef __cplusplus
extern "C" {
#endif

struct robotPoint {
    double x,
    double y
};

void setCurrentLocation(double x, double y);
void setCurrentX(double x);
void setCurrentY(double y);

void followPath(struct robotPoint* points, int size, double endSpeed, void (*endCallback)(void));

#ifdef __cplusplus
}
#endif

#endif
