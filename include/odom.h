#ifndef _PROS_ODOM_H_
#define _PROS_ODOM_H_

static double lastTheta = 0;
void recordPosition();
void recordPosition2();
void testThread();
void movePosition(double targetX, double targetY, bool faceBack);
#endif // _PROS_ODOM_H_