/* This library allows the robot to follow a path of discrete points.
 * The robot will move in straight line along the edges and turn on itself on
 * the vertices of the path.
 * A function can be called when the end of the path is reached.
 * The user can also specify the speed of the robot at the end of the path.
 *
 * /!\ WARNING : this library depends on librobotdriver. You HAVE TO call
 * the library's initMotionController() before any operations
 */

#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#ifdef __cplusplus
extern "C" {
#endif

/* a point on the game table, with coordinates in mm (can be positive and negative)
 * the user is free to place to origin as it doesn't affect this lib at all */
struct robotPoint {
    double x;
    double y;
};

/* set the current robot absolute location. By default, the robot starts at (0,0)
 * x, y: coordinates in mm */
void setCurrentLocation(double x, double y);
/* separate setting of the coordinates. Can be useful when recalibrating the robot
 * x, y: coordinates in mm */
void setCurrentX(double x);
void setCurrentY(double y);

/* set the speed (absolute value) of the speed the robot will travel at
 * speed: the cruise speed in m/s */
void setCruiseSpeed(double speed);

/* Move the robot along the given path.
 * points : a list of points the robot has to pass by, in absolute coordinates (in mm)
 * size : the total number of points in points
 * endSpeed : the absolute value of the speed the robot should have when it
        reaches the end of the path (in m/s)
 * endCallback : a function called when the end of the path is reached, or NULL
        please never block or do long processing in callbacks, it will affect
        the whole driver mecanism and may cause erratic behavior. */
void followPath(struct robotPoint* points, int size, double endSpeed, void (*endCallback)(void));

void ffollow(const char * pathName, void (*endCallback)(void));

#ifdef __cplusplus
}
#endif

#endif
