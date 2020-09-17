#include "simulator.h"
#include "world.h"

int main()
{
    RobotsFormation *forms = new RobotsFormation(-2);
    World *world = new World(forms);
    std::cout << world->robots[0]->on << std::endl;
    return 0;
}