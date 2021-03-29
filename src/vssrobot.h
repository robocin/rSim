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

#ifndef VSSROBOT_H
#define VSSROBOT_H

#include "physics/pworld.h"
#include "physics/pcylinder.h"
#include "physics/pbox.h"
#include "physics/pball.h"
#include "vssconfig.h"

#define ROBOT_GRAY 0.4

class VSSRobot
{
    PWorld *physics;
    PBall *m_ball;
    dReal m_x, m_y, m_z;
    dReal m_r, m_g, m_b;
    dReal m_dir;
    int m_rob_id;
    bool last_state{};

public:
    dSpaceID space;
    PObject *chassis;
    PBox *boxes[3]{};
    bool on;
    //these values are not controled by this class
    bool selected{};
    dReal select_x{}, select_y{}, select_z{};
    class Wheel
    {
    public:
        int id;
        Wheel(VSSRobot *robot, int _id, dReal ang, dReal ang2);
        void step();
        ~Wheel();
        dJointID joint;
        dJointID motor;
        PCylinder *cyl;
        dReal desiredAngularSpeed, maxAngularSpeed; // rad/s
        VSSRobot *rob;
    } * wheels[2]{};

    class RBall
    {
    public:
        int id;
        RBall(VSSRobot *robot, int _id, dReal ang, dReal ang2);
        void step();
        ~RBall();
        dJointID joint;
        PBall *pBall;
        VSSRobot *rob;
    } * balls[4]{};

    VSSRobot(PWorld *world, PBall *ball, dReal x, dReal y, dReal z,
           int rob_id, int dir, bool turn_on);
    ~VSSRobot();
    void step();
    void setWheelDesiredAngularSpeed(int i, dReal s); //i = 0,1,2,3
    void setSpeed(dReal vx, dReal vy, dReal vw);
    dReal getSpeed(int i);
    void incSpeed(int i, dReal v);
    void resetSpeeds();
    void resetRobot();
    void getXY(dReal &x, dReal &y);
    dReal getDir(dReal &k);
    void setXY(dReal x, dReal y);
    void setDir(dReal ang);
    int getID();
    PBall *getBall();
    PWorld *getWorld();
};

#define ROBOT_START_Z() (VSSConfig::Robot().getHeight() * 0.5 + VSSConfig::Robot().getBottomHeight())

#endif // ROBOT_H
