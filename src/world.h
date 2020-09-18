/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef WORLD_H
#define WORLD_H

#include <QUdpSocket>
#include <QList>

#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"
#include "physics/pray.h"

#include "net/robocup_ssl_server.h"

#include "robot.h"

#include "speed_estimator.h"
#define WALL_COUNT 16
#define MAX_ROBOT_COUNT 12 //don't change
#define TEAM_COUNT 2

class RobotsFormation;
class SendingPacket
{
public:
    SendingPacket(fira_message::sim_to_ref::Environment *_packet, int _t);
    fira_message::sim_to_ref::Environment *packet;
    int t;
};

class World
{
private:
    int frame_num;
    dReal last_dt;
    QList<SendingPacket *> sendQueue;
    char *in_buffer;
    bool lastInfraredState[TEAM_COUNT][MAX_ROBOT_COUNT]{};
    int steps_super, steps_fault;
    KickStatus lastKickState[TEAM_COUNT][MAX_ROBOT_COUNT]{};

    void getValidPosition(dReal &x, dReal &y, uint32_t max);

public:
    dReal customDT;
    int goals_yellow = 0;
    int goals_blue = 0;
    World();
    ~World();
    void simStep(dReal dt = -1);
    void step(dReal dt = -1);
    void posProcess();
    fira_message::sim_to_ref::Environment *generatePacket();
    void sendVisionBuffer();
    int robotIndex(unsigned int robot, int team);
    const dReal *ball_vel;
    const dReal *robot_vel;
    const dReal *robot_angular_vel;

    PWorld *physics;
    PBall *ball;
    speedEstimator *ball_speed_estimator;
    speedEstimator *blue_speed_estimator[MAX_ROBOT_COUNT];
    speedEstimator *yellow_speed_estimator[MAX_ROBOT_COUNT];
    PGround *ground;
    PRay *ray;
    PFixedBox *walls[WALL_COUNT]{};
    int selected{};
    bool show3DCursor;
    dReal cursor_x{}, cursor_y{}, cursor_z{};
    dReal cursor_radius{};
    RoboCupSSLServer *visionServer{};
    QUdpSocket *commandSocket{};
    bool updatedCursor;
    bool withGoalKick = false;
    bool randomStart = false;
    CRobot *robots[MAX_ROBOT_COUNT * 2]{};
    QElapsedTimer *timer, *timer_fault;
    bool received = true;
    bool fullSpeed = false;
    int minute = 0;
    dReal last_speed = 0.0;
    std::pair<float, float> ball_prev_pos = std::pair<float, float>(0.0, 0.0);
public slots:
    void recvActions();
};

class RobotsFormation
{
public:
    dReal x[MAX_ROBOT_COUNT]{};
    dReal y[MAX_ROBOT_COUNT]{};
    RobotsFormation(int type);
    void setAll(const dReal *xx, const dReal *yy);
    void loadFromFile(const QString &filename);
    void resetRobots(CRobot **r, int team);
};

dReal fric(dReal f);

#endif // World_H
