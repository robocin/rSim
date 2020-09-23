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
#include "physics/pray.h"

#include "robot.h"

#include "speed_estimator.h"
#define WALL_COUNT 16
#define MAX_ROBOT_COUNT 12 //don't change
#define TEAM_COUNT 2
#define STATE_SIZE 29 // BALL_XYZ, BALLV_XY, 3*(RBLUE_XY, RBLUEV_XY), 3*(RYELLOW_XY, RYELLOWV_XY) 

class RobotsFormation;

class World
{
private:
    int frame_num;
    char *in_buffer;
    bool lastInfraredState[TEAM_COUNT][MAX_ROBOT_COUNT]{};
    int episodeSteps, steps_fault;
    dReal last_dt;
    std::vector<double> state = std::vector<double>(static_cast<std::size_t>(STATE_SIZE));
    KickStatus lastKickState[TEAM_COUNT][MAX_ROBOT_COUNT]{};

    void getValidPosition(dReal &x, dReal &y, uint32_t max);

public:
    int goalsYellow = 0;
    int goalsBlue = 0;
    int minute = 0;
    int selected{};
    bool updatedCursor;
    bool withGoalKick = false;
    bool randomStart = false;
    bool show3DCursor;
    bool received = true;
    bool fullSpeed = false;
    std::pair<float, float> ball_prev_pos = std::pair<float, float>(0.0, 0.0);
    PWorld *physics;
    PBall *ball;
    speedEstimator *ball_speed_estimator;
    speedEstimator *blue_speed_estimator[MAX_ROBOT_COUNT];
    speedEstimator *yellow_speed_estimator[MAX_ROBOT_COUNT];
    PGround *ground;
    PRay *ray;
    PFixedBox *walls[WALL_COUNT]{};
    dReal cursor_x{}, cursor_y{}, cursor_z{};
    dReal cursor_radius{};
    CRobot *robots[MAX_ROBOT_COUNT * 2]{};
    QElapsedTimer *timer, *timer_fault;
    dReal last_speed = 0.0;

    World();
    ~World();
    void simStep(dReal dt = -1);
    void step(dReal dt, std::vector<std::tuple<double, double>> actions);
    void posProcess();

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
    \brief get current episode time in seconds
    \return int, representing episode time in seconds
    */
    int getEpisodeTime();

    /**
    \brief State has [ballX, ballY, ballZ, ballVx, ballVy,
                    robotBlueX, robotBlueY, robotBlueVx, robotBlueVy,
                    robotYellowX, robotYellowY, robotYellowVx, robotYellowVy]
    \return return std::vector of float representing the state
    */
    const std::vector<double>& getState();

    int robotIndex(unsigned int robot, int team);
    void setActions(std::vector<std::tuple<double, double>> actions);
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
