#include "simulator.h"
#include "world.h"
#include "config.h"
#include <chrono> 

const std::vector<double>&step(World *world)
{
    std::vector<std::tuple<double, double>> actions;
    actions.push_back(std::make_tuple(1.0, 1.0));
    world->step(Config::World().getDeltaTime(), actions);
    return world->getState();
}

int main()
{
    World *world = new World();
    while (true)
    {
        const std::vector<double> state = step(world);
        std::cout << "robot blue x " << state[5] << " robot blue y " << state[6] << '\n';
    }
    return 0;
}
