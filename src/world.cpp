#include "world.h"

VSSWorld::VSSWorld()
{
    double dt = 0.05;
    double gravity = 9.8;
    int robotCount = 0;
    this->physics = new PWorld(dt, gravity, robotCount);
}
