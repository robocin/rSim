#include "simulator.h"
#include "world.h"
#include "config.h"
#include <chrono> 

bool step(World *world)
{
    world->recvActions();
    world->step(Config::World().getDeltaTime());
}

int main()
{
    World *world = new World();
    RoboCupSSLServer *visionServer = nullptr;
    QUdpSocket *commandSocket = nullptr;
    commandSocket = new QUdpSocket(nullptr);
    commandSocket->bind(QHostAddress::Any, Config::Communication().getCommandListenPort());
    visionServer = new RoboCupSSLServer();
    visionServer->change_address(Config::Communication().getVisionMulticastAddr());
    visionServer->change_port(Config::Communication().getVisionMulticastPort());
    world->visionServer = visionServer;
    world->commandSocket = commandSocket;
    while (true)
    {
        step(world);
    }
    return 0;
}
