#include "simulator.h"
#include "world.h"
#include "config.h"
#include <chrono> 

const std::vector<double>&step(World *world)
{
    world->step(Config::World().getDeltaTime());
    return world->getState();
}

int main()
{
    World *world = new World();
    while (true)
    {
        const std::vector<double> state = step(world);
    }
    return 0;
}
