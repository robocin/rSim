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

#ifndef SSLWORLD_H
#define SSLWORLD_H


#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"

#include "sslrobot.h"
#include "utils.h"

#define WALL_COUNT 10
#define MAX_ROBOT_COUNT 22 //don't change

class SSLWorld
{
private:
    double timeStep;
    int stateSize;
    std::vector<double> state;

public:
    bool fullSpeed = false;
    PWorld *physics;
    PBall *ball;
    PGround *ground;
    PFixedBox *walls[WALL_COUNT]{};
    SSLRobot *robots[MAX_ROBOT_COUNT * 2]{};
    SSLConfig::Field field = SSLConfig::Field();

    SSLWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, double timeStep,
             double *ballPos, double *blueRobotsPos, double *yellowRobotsPos);
    ~SSLWorld();
    void step(std::vector<double*> actions);
    void replace(double *ball_pos, double *pos_blue, double *pos_yellow);
    void initWalls();
    int getNumRobotsBlue() { return this->field.getRobotsBlueCount(); }
    int getNumRobotsYellow() { return this->field.getRobotsYellowCount(); }
    const std::vector<double> getFieldParams();
    const std::vector<double> &getState();

    void setActions(std::vector<double*> actions);
};

#endif // SSLWorld_H
