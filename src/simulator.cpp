#include "simulator.h"
#include "world.h"
#include "config.h"

bool step(World *world)
{

    world->step(Config::World().getDeltaTime());
}

int main()
{
    RobotsFormation *forms = new RobotsFormation(-2);
    World *world = new World(forms);
    RoboCupSSLServer *visionServer = nullptr;
    QUdpSocket *commandSocket = nullptr;
    commandSocket = new QUdpSocket(nullptr);
    commandSocket->bind(QHostAddress::Any, Config::Communication().getCommandListenPort());
    visionServer = new RoboCupSSLServer();
    visionServer->change_address(Config::Communication().getVisionMulticastAddr());
    visionServer->change_port(Config::Communication().getVisionMulticastPort());
    world->visionServer = visionServer;
    world->commandSocket = commandSocket;
    world->step(0.016);
    return 0;
}
