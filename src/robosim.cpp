#include "sslworld.h"


int main()
{
    double ballPos[2] = {0., 0.};
    double blueRobotsPos[3] = {0., 0., 0.};
    double yellowRobotsPos[] = {};

    auto world = new SSLWorld(1, 1, 0, 16 / 1000.0, ballPos, blueRobotsPos, yellowRobotsPos);

}