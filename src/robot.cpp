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

#include "robot.h"

// ang2 = position angle
// ang  = rotation angle
CRobot::Wheel::Wheel(CRobot *robot, int _id, dReal ang, dReal ang2, int wheeltexid)
{
    id = _id;
    rob = robot;
    dReal rad = Config::Robot().getRadius() + Config::Robot().getWheelThickness() / 2.0;
    ang *= M_PI / 180.0f;
    ang2 *= M_PI / 180.0f;
    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + rad * cos(ang2);
    dReal centery = y + rad * sin(ang2);
    dReal centerz = z - Config::Robot().getRadius() * 0.5 + Config::Robot().getWheelRadius() - Config::Robot().getBottomHeight();
    cyl = new PCylinder(centerx, centery, centerz, Config::Robot().getWheelRadius(), Config::Robot().getWheelThickness(), Config::Robot().getWheelMass(), wheeltexid);
    cyl->setRotation(-sin(ang), cos(ang), 0, M_PI * 0.5);
    cyl->setBodyRotation(-sin(ang), cos(ang), 0, M_PI * 0.5, true);    //set local rotation matrix
    cyl->setBodyPosition(centerx - x, centery - y, centerz - z, true); //set local position vector
    cyl->space = rob->space;

    rob->physics->addObject(cyl);

    joint = dJointCreateHinge(rob->physics->world, nullptr);

    dJointAttach(joint, rob->chassis->body, cyl->body);
    const dReal *a = dBodyGetPosition(cyl->body);
    dJointSetHingeAxis(joint, cos(ang), sin(ang), 0);
    dJointSetHingeAnchor(joint, a[0], a[1], a[2]);

    motor = dJointCreateAMotor(rob->physics->world, nullptr);
    dJointAttach(motor, rob->chassis->body, cyl->body);
    dJointSetAMotorNumAxes(motor, 1);
    dJointSetAMotorAxis(motor, 0, 1, cos(ang), sin(ang), 0);
    dJointSetAMotorParam(motor, dParamFMax, Config::Robot().getWheelMotorFMax());
    speed = 0;
}

void CRobot::Wheel::step()
{
    dJointSetAMotorParam(motor, dParamVel, speed);
    dJointSetAMotorParam(motor, dParamFMax, Config::Robot().getWheelMotorFMax());
}

CRobot::RBall::RBall(CRobot *robot, int _id, dReal ang, dReal ang2)
{
    id = _id;
    rob = robot;
    dReal rad = Config::Robot().getRadius() - Config::World().getBallRadius();
    ang *= M_PI / 180.0f;
    ang2 *= M_PI / 180.0f;
    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + rad * cos(ang2);
    dReal centery = y + rad * sin(ang2);
    dReal centerz = z - Config::Robot().getRadius() * 0.5 + Config::World().getBallRadius() - Config::Robot().getBottomHeight();
    pBall = new PBall(centerx, centery, centerz, Config::World().getBallRadius(), Config::World().getBallMass());
    pBall->setRotation(-sin(ang), cos(ang), 0, M_PI * 0.5);
    pBall->setBodyRotation(-sin(ang), cos(ang), 0, M_PI * 0.5, true);    //set local rotation matrix
    pBall->setBodyPosition(centerx - x, centery - y, centerz - z, true); //set local position vector
    pBall->space = rob->space;

    rob->physics->addObject(pBall);

    joint = dJointCreateHinge(rob->physics->world, nullptr);

    dJointAttach(joint, rob->chassis->body, pBall->body);
    const dReal *a = dBodyGetPosition(pBall->body);
    dJointSetHingeAxis(joint, cos(ang), sin(ang), 0);
    dJointSetHingeAnchor(joint, a[0], a[1], a[2]);

    speed = 0;
}

CRobot::CRobot(PWorld *world, PBall *ball, dReal x, dReal y, dReal z,
               int rob_id, int wheeltexid, int dir, bool turn_on)
{
    m_x = x;
    m_y = y;
    m_z = z;
    physics = world;
    m_ball = ball;
    m_dir = dir;
    m_rob_id = rob_id;

    space = physics->space;

    chassis = new PBox(x, y, z, Config::Robot().getRadius() * 2, Config::Robot().getRadius() * 2, Config::Robot().getHeight(), Config::Robot().getBodyMass() * 0.99f, rob_id, true);
    chassis->space = space;
    physics->addObject(chassis);

    dummy = new PBox(x, y, z, Config::Robot().getRadius() * 2, Config::Robot().getRadius() * 2, Config::Robot().getHeight(), Config::Robot().getBodyMass() * 0.99f, rob_id, true);
    dummy->space = space;
    physics->addObject(dummy);

    dummy_to_chassis = dJointCreateFixed(world->world, nullptr);
    dJointAttach(dummy_to_chassis, chassis->body, dummy->body);

    wheels[0] = new Wheel(this, 0, Config::Robot().getWheel1Angle(), Config::Robot().getWheel1Angle(), wheeltexid);
    wheels[1] = new Wheel(this, 1, Config::Robot().getWheel2Angle(), Config::Robot().getWheel2Angle(), wheeltexid);
    balls[0] = new RBall(this, 0, Config::Robot().getWheel1Angle() + 90, Config::Robot().getWheel1Angle() + 90);
    balls[1] = new RBall(this, 1, Config::Robot().getWheel2Angle() + 90, Config::Robot().getWheel2Angle() + 90);
    firsttime = true;
    on = turn_on;
}

CRobot::~CRobot() = default;

PBall *CRobot::getBall()
{
    return m_ball;
}

PWorld *CRobot::getWorld()
{
    return physics;
}

int CRobot::getID()
{
    return m_rob_id - 1;
}

void normalizeVector(dReal &x, dReal &y, dReal &z)
{
    dReal d = sqrt(x * x + y * y + z * z);
    x /= d;
    y /= d;
    z /= d;
}

void CRobot::step()
{
    if (on)
    {
        if (firsttime)
        {
            if (m_dir == -1)
                setDir(180);
            firsttime = false;
        }
    }
    else
    {
        if (last_state)
            wheels[0]->speed = wheels[1]->speed = 0;
    }
    for (auto &wheel : wheels)
        wheel->step();
    last_state = on;
}

void CRobot::resetSpeeds()
{
    wheels[0]->speed = wheels[1]->speed = 0;
}

void CRobot::resetRobot()
{
    resetSpeeds();
    dBodySetLinearVel(chassis->body, 0, 0, 0);
    dBodySetAngularVel(chassis->body, 0, 0, 0);
    dBodySetLinearVel(dummy->body, 0, 0, 0);
    dBodySetAngularVel(dummy->body, 0, 0, 0);
    for (auto &wheel : wheels)
    {
        dBodySetLinearVel(wheel->cyl->body, 0, 0, 0);
        dBodySetAngularVel(wheel->cyl->body, 0, 0, 0);
    }
    dReal x, y;
    getXY(x, y);
    setXY(x, y);
    if (m_dir == -1)
        setDir(180);
    else
        setDir(0);
}

void CRobot::getXY(dReal &x, dReal &y)
{
    dReal xx, yy, zz;
    chassis->getBodyPosition(xx, yy, zz);
    x = xx;
    y = yy;
}

dReal CRobot::getDir()
{
    dReal x, y, z;
    chassis->getBodyDirection(x, y, z);
    dReal dot = x; //zarb dar (1.0,0.0,0.0)
    dReal length = sqrt(x * x + y * y);
    auto absAng = (dReal)(acos((dReal)(dot / length)) * (180.0f / M_PI));
    return (y > 0) ? absAng : -absAng;
}

dReal CRobot::getDir(dReal &k)
{
    dReal x, y, z;
    chassis->getBodyDirection(x, y, z, k);
    dReal dot = x; //zarb dar (1.0,0.0,0.0)
    dReal length = sqrt(x * x + y * y);
    auto absAng = (dReal)(acos((dReal)(dot / length)) * (180.0f / M_PI));
    return (y > 0) ? absAng : -absAng;
}

void CRobot::setXY(dReal x, dReal y)
{
    dReal xx, yy, zz, kx, ky, kz;
    dReal height = ROBOT_START_Z();
    chassis->getBodyPosition(xx, yy, zz);
    chassis->setBodyPosition(x, y, height);
    dummy->setBodyPosition(x, y, height);
    for (auto &wheel : wheels)
    {
        wheel->cyl->getBodyPosition(kx, ky, kz);
        wheel->cyl->setBodyPosition(kx - xx + x, ky - yy + y, kz - zz + height);
    }
}

void CRobot::setDir(dReal ang)
{
    ang *= M_PI / 180.0f;
    chassis->setBodyRotation(0, 0, 1, ang);
    dummy->setBodyRotation(0, 0, 1, ang);
    dMatrix3 wLocalRot, wRot, cRot;
    dVector3 localPos, finalPos, cPos;
    chassis->getBodyPosition(cPos[0], cPos[1], cPos[2], false);
    chassis->getBodyRotation(cRot, false);
    dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
    finalPos[0] += cPos[0];
    finalPos[1] += cPos[1];
    finalPos[2] += cPos[2];
    for (auto &wheel : wheels)
    {
        wheel->cyl->getBodyRotation(wLocalRot, true);
        dMultiply0(wRot, cRot, wLocalRot, 3, 3, 3);
        dBodySetRotation(wheel->cyl->body, wRot);
        wheel->cyl->getBodyPosition(localPos[0], localPos[1], localPos[2], true);
        dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
        finalPos[0] += cPos[0];
        finalPos[1] += cPos[1];
        finalPos[2] += cPos[2];
        wheel->cyl->setBodyPosition(finalPos[0], finalPos[1], finalPos[2], false);
    }
}

void CRobot::setSpeed(int i, dReal s)
{
    if (!((i >= 2) || (i < 0)))
        wheels[i]->speed = s;
}

void CRobot::setSpeed(dReal vx, dReal vy, dReal vw)
{
    // Calculate Motor Speeds
    dReal dw1 = (1.0 / Config::Robot().getWheelRadius()) * ((Config::Robot().getRadius() * vw) + vx);
    dReal dw2 = (1.0 / Config::Robot().getWheelRadius()) * ((Config::Robot().getRadius() * -vw) + vx);

    setSpeed(0, dw1);
    setSpeed(1, dw2);
}

dReal CRobot::getSpeed(int i)
{
    if ((i >= 2) || (i < 0))
        return -1;
    return wheels[i]->speed;
}

void CRobot::incSpeed(int i, dReal v)
{
    if (!((i >= 2) || (i < 0)))
        wheels[i]->speed += v;
}
