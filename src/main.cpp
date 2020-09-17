#include "main.h"
#include "world.h"

int main()
{
    double dt = 0.05;
    double gravity = 9.8;
    int robotCount = 0;
    PWorld *physics = new PWorld(dt, gravity, robotCount);
    return 0;
}