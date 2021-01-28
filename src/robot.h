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

#ifndef ROBOT_H
#define ROBOT_H

#include "physics/pworld.h"
#include "physics/pcylinder.h"
#include "physics/pbox.h"
#include "physics/pball.h"
#include "config.h"

enum KickStatus {
    NO_KICK   = 0,
    FLAT_KICK = 1,
    CHIP_KICK = 2,
};

class SSLRobot
{
    PWorld *physics;
    PBall *ball;
    int rob_id;

public:
    dReal _x, _y, _z;
    dReal _dir;
    dSpaceID space;
    PObject *chassis;
    PBall* dummy;
    dJointID dummy_to_chassis;

    class Wheel
    {
    public:
        int id;
        Wheel(CRobot *robot, int _id, dReal ang, dReal ang2);
        void step();
        dJointID joint;
        dJointID motor;
        PCylinder *cyl;
        dReal desiredAngularSpeed; // Degrees/s
        CRobot *rob;
    } * wheels[4]{};
    class Kicker
    {
      private:
        KickStatus kicking;
        int rolling;
        int kickstate;
        dReal m_kickspeed,m_kicktime;
        bool holdingBall;
      public:
        Kicker(Robot* robot);
        void step();
        void kick(dReal kickspeedx, dReal kickspeedz);
        void setRoller(int roller);
        int getRoller();
        void toggleRoller();
        bool isTouchingBall();
        KickStatus isKicking();
        void holdBall();
        void unholdBall();
        dJointID joint;
        dJointID robot_to_ball;
        PBox* box;
        Robot* rob;
    } *kicker;

    SSLRobot(PWorld *world, PBall *ball, dReal x, dReal y, dReal z,
           int robot_id, dReal dir);
    ~SSLRobot();
    void step();
    void setWheelDesiredAngularSpeed(int i, dReal s); //i = 0,1,2,3
    void setSpeed(dReal vx, dReal vy, dReal vw);
    void incSpeed(int i, dReal v);
    void resetSpeeds();
    void resetRobot();
    void getXY(dReal &x, dReal &y);
    dReal getDir();
    dReal getDir(dReal &k);
    void setXY(dReal x, dReal y);
    void setDir(dReal ang);
    int getID();
    PBall *getBall();
    PWorld *getWorld();
};

#define ROBOT_START_Z() (Config::Robot().getHeight() * 0.5 + Config::Robot().getBottomHeight())

#endif // ROBOT_H
