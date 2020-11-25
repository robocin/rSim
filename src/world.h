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

#include <QList>
#include <QElapsedTimer>

#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"

#include "robot.h"
#include "utils.h"

#define WALL_COUNT 16
#define MAX_ROBOT_COUNT 12 //don't change
#define TEAM_COUNT 2
#define STATE_SIZE 41 // BALL_XYZ, BALLV_XY, 3*(RBLUE_XY, RBLUEV_XY), 3*(RYELLOW_XY, RYELLOWV_XY)

class World
{
private:
    int episodeSteps;
    double timeStep;
    std::vector<double> state = std::vector<double>(static_cast<std::size_t>(STATE_SIZE));
    Config::Field field = Config::Field();

public:
    int goalsYellow = 0;
    int goalsBlue = 0;
    int minute = 0;
    bool withGoalKick = false;
    bool randomStart = false;
    bool fullSpeed = false;
    std::pair<float, float> ball_prev_pos = std::pair<float, float>(0.0, 0.0);
    PWorld *physics;
    PBall *ball;
    PGround *ground;
    PFixedBox *walls[WALL_COUNT]{};
    dReal cursor_x{}, cursor_y{}, cursor_z{};
    dReal cursor_radius{};
    CRobot *robots[MAX_ROBOT_COUNT * 2]{};
    QElapsedTimer *timer, *timer_fault;
    dReal last_speed = 0.0;

    World(int fieldType, int nRobotsBlue, int nRobotsYellow, double timeStep);
    ~World();
    void simStep(dReal dt = -1);
    void step(dReal dt, std::vector<std::tuple<double, double>> actions);
    void replace(double *ball_pos, double *pos_blue, double *pos_yellow);
    void replace_with_vel(double *ball_pos, double *pos_blue, double *pos_yellow);
    void initWalls();
    int getNumRobotsBlue() { return this->field.getRobotsBlueCount(); }
    int getNumRobotsYellow() { return this->field.getRobotsYellowCount(); }
    double getTimeStep() { return this->timeStep; }

    /**
    \brief goals has [blueTeamGoals, YellowTeamGoals]
    \return return std::vector of int representing current episode goals count
    */
    const std::vector<int> getGoals();

    /**
    \brief FieldParams has [FieldWidth, FieldLenght, GoalDepth, GoalWidth]
    \return return std::vector of double representing field parameters
    */
    const std::vector<double> getFieldParams();

    /**
    \brief get current episode time in miliseconds
    \return int, representing episode time in miliseconds
    */
    int getEpisodeTime();

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
