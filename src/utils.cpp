#include "utils.h"

dReal smallestAngleDiff(dReal angle, dReal last_angle)
{
    // TODO Assert as false if the robot is capable of rotating more than 180
    // degrees in one time step
    dReal diff = (angle - last_angle);
    
    if ((diff + 180.0) > 360.0)
    {
        diff = diff - 360.0;
    } else if ((diff - 180.0) < -360.0)
    {
        diff = diff + 360.0;
    }
    
    return diff;
}

dReal fric(dReal f)
{
    if (f + 1 < 0.001)
        return dInfinity;
    return f;
}
