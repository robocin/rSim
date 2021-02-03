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

#ifndef VSSWORLD_H
#define VSSWORLD_H


#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"

#include "vssrobot.h"
#include "utils.h"

#define WALL_COUNT 16
#define MAX_ROBOT_COUNT 12 //don't change

class VSSWorld
{
private:
    double timeStep;
    int stateSize;
    std::vector<double> state;

public:
    VSSConfig::Field field = VSSConfig::Field();
    bool fullSpeed = false;
    PWorld *physics;
    PBall *ball;
    PGround *ground;
    PFixedBox *walls[WALL_COUNT]{};
    dReal cursor_x{}, cursor_y{}, cursor_z{};
    dReal cursor_radius{};
    CRobot *robots[MAX_ROBOT_COUNT * 2]{};
    dReal last_speed = 0.0;

    VSSWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, double timeStep,
             double *ballPos, double *blueRobotsPos, double *yellowRobotsPos);
    ~VSSWorld();
    void simStep(dReal dt = -1);
    void step(dReal dt, std::vector<std::tuple<double, double>> actions);
    void replace(double *ball_pos, double *pos_blue, double *pos_yellow);
    void replace_with_vel(double *ball_pos, double *pos_blue, double *pos_yellow);
    void initWalls();
    int getNumRobotsBlue() { return this->field.getRobotsBlueCount(); }
    int getNumRobotsYellow() { return this->field.getRobotsYellowCount(); }
    double getTimeStep() { return this->timeStep; }

    /**
    \brief FieldParams has [FieldWidth, FieldLenght, GoalDepth, GoalWidth]
    \return return std::vector of double representing field parameters
    */
    const std::vector<double> getFieldParams();

    /**
    \brief State has [ballX, ballY, ballZ, ballVx, ballVy,
                    robotBlueX, robotBlueY, robotBlueVx, robotBlueVy,
                    robotYellowX, robotYellowY, robotYellowVx, robotYellowVy]
    \return return std::vector of float representing the state
    */
    const std::vector<double> &getState();

    int robotIndex(unsigned int robot, int team);
    void setActions(std::vector<std::tuple<double, double>> actions);
};

dReal fric(dReal f);

#endif // World_H
