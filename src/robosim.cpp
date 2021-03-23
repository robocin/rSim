#include "sslworld.h"
// #include <ode/ode.h>

// void test() {
//     dVector3 local_Pos{};
// }

int main()
{
    double ballPos[2] = {0., 0.};
    double blueRobotsPos[3] = {0., 0., 0.};
    double yellowRobotsPos[] = {};

    auto world = new SSLWorld(1, 1, 0, 16 / 1000.0, ballPos, blueRobotsPos, yellowRobotsPos);
    delete world;

    return 0;
}