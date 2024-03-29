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

#ifndef PWORLD_H
#define PWORLD_H

#include <vector>
#include "pobject.h"

class PSurface;
class PWorld
{
private:
    dJointGroupID contactgroup;
    std::vector<PObject *> objects;
    std::vector<PSurface *> surfaces;
    dReal delta_time;
    int **sur_matrix;
    int objects_count;

public:
    PWorld(dReal dt, dReal gravity, int robot_count);
    ~PWorld();
    void setGravity(dReal gravity);
    int addObject(PObject* o);
    int addBallObject(PObject* o);
    int addGroundObject(PObject* o);
    int addWallObject(PObject* o);
    int addWheelObject(PObject* o);
    int addChassisObject(PObject* o);
    int addKickerObject(PObject* o);
    void initAllObjects();
    PSurface *createSurface(PObject *o1, PObject *o2);
    PSurface* createOneWaySurface(PObject* o1,PObject* o2);
    PSurface *findSurface(PObject *o1, PObject *o2);
    void step(dReal dt = -1, bool sync = false);
    void handleCollisions(dGeomID o1, dGeomID o2);
    dWorldID world;
    dSpaceID space, spaceChassis, spaceKicker, spaceWall, spaceWheel;
    PObject *ball, *ground;
    int robot_count;
};

typedef bool PSurfaceCallback(dGeomID o1, dGeomID o2, PSurface *s, int robot_count);
class PSurface
{
public:
    PSurface();
    dSurfaceParameters surface{};
    bool isIt(dGeomID i1, dGeomID i2);
    dGeomID id1{}, id2{};
    bool usefdir1;    //if true use fdir1 instead of ODE value
    dVector3 fdir1{}; //fdir1 is a normalized vector tangent to friction force vector
    dVector3 contactPos{}, contactNormal{};
    PSurfaceCallback *callback;
};
#endif // PWORLD_H
