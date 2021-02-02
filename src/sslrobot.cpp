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

#include "sslrobot.h"

// ang2 = position angle
// ang  = rotation angle
SSLRobot::Wheel::Wheel(SSLRobot *robot, int _id, dReal ang, dReal ang2)
{
    this->id = _id;
    this->rob = robot;
    dReal rad = Config::Robot().getRadius() + Config::Robot().getWheelThickness() / 2.0;
    ang *= M_PI / 180.0f;
    ang2 *= M_PI / 180.0f;
    dReal x = this->rob->_x;
    dReal y = this->rob->_y;
    dReal z = this->rob->_z;
    dReal centerx = x + rad * cos(ang2);
    dReal centery = y + rad * sin(ang2);
    dReal centerz = z - Config::Robot().getHeight() * 0.5 + Config::Robot().getWheelRadius() - Config::Robot().getBottomHeight();
    this->cyl = new PCylinder(centerx, centery, centerz, Config::Robot().getWheelRadius(), Config::Robot().getWheelThickness(), Config::Robot().getWheelMass());
    this->cyl->setRotation(-sin(ang), cos(ang), 0, M_PI * 0.5);
    this->cyl->setBodyRotation(-sin(ang), cos(ang), 0, M_PI * 0.5, true);    //set local rotation matrix
    this->cyl->setBodyPosition(centerx - x, centery - y, centerz - z, true); //set local position vector
    this->cyl->space = this->rob->space;

    this->rob->physics->addObject(this->cyl);

    this->joint = dJointCreateHinge(this->rob->physics->world, nullptr);

    dJointAttach(this->joint, this->rob->chassis->body, this->cyl->body);
    const dReal *a = dBodyGetPosition(this->cyl->body);
    dJointSetHingeAxis(this->joint, cos(ang), sin(ang), 0);
    dJointSetHingeAnchor(this->joint, a[0], a[1], a[2]);

    this->motor = dJointCreateAMotor(this->rob->physics->world, nullptr);
    dJointAttach(this->motor, this->rob->chassis->body, this->cyl->body);
    dJointSetAMotorNumAxes(this->motor, 1);
    dJointSetAMotorAxis(this->motor, 0, 1, cos(ang), sin(ang), 0);
    dJointSetAMotorParam(this->motor, dParamFMax, Config::Robot().getWheelMotorMaxTorque());
    this->desiredAngularSpeed = 0;
}

void SSLRobot::Wheel::step()
{
    dJointSetAMotorParam(this->motor, dParamVel, this->desiredAngularSpeed);
    dJointSetAMotorParam(this->motor, dParamFMax, Config::Robot().getWheelMotorMaxTorque());
}

SSLRobot::Kicker::Kicker(SSLRobot* robot) : holdingBall(false)
{
    this->rob = robot;

    dReal x = this->rob->_x;
    dReal y = this->rob->_y;
    dReal z = this->rob->_z;

    dReal centerX = x + (Config::Robot().getDistanceCenterKicker() + Config::Robot().getKickerThickness());
    dReal centerY = y;
    dReal centerZ = z - (Config::Robot().getHeight()) * 0.5f + Config::Robot().getWheelRadius() - Config::Robot().getBottomHeight() + Config::Robot().getKickerZ();
    
    this->box = new PBox(
        centerX, centerY, centerZ, 
        Config::Robot().getKickerThickness(), Config::Robot().getKickerWidth(),
        Config::Robot().getKickerHeight(),Config::Robot().getKickerMass()
        );
    this->box->setBodyPosition(centerX - x, centerY - y, centerZ - z, true);
    this->box->space = this->rob->space;

    this->rob->physics->addObject(this->box);

    this->joint = dJointCreateHinge(this->rob->physics->world, 0);
    dJointAttach(this->joint,this->rob->chassis->body,this->box->body);
    
    const dReal *aa = dBodyGetPosition(this->box->body);
    dJointSetHingeAnchor(this->joint, aa[0], aa[1], aa[2]);
    dJointSetHingeAxis(this->joint, 0, -1,0);

    dJointSetHingeParam(this->joint,dParamVel,0);
    dJointSetHingeParam(this->joint,dParamLoStop,0);
    dJointSetHingeParam(this->joint,dParamHiStop,0);

    this->rolling = 0;
    this->kicking = NO_KICK;
}

void SSLRobot::Kicker::step()
{
    if (!isTouchingBall() || this->rolling == 0) unholdBall();
    if (this->kicking != NO_KICK)
    {
        this->kickstate--;
        if (this->kickstate<=0) this->kicking = NO_KICK;
    }
    else if (this->rolling !=0)
    {
        if (isTouchingBall())
        {
            holdBall();
        }
    }
}

bool SSLRobot::Kicker::isTouchingBall()
{
    dReal vx,vy,vz;
    dReal bx,by,bz;
    dReal kx,ky,kz;

    this->rob->chassis->getBodyDirection(vx,vy,vz);
    this->rob->getBall()->getBodyPosition(bx,by,bz);
    this->box->getBodyPosition(kx,ky,kz);

    kx += vx * Config::Robot().getKickerThickness()*0.5f;
    ky += vy * Config::Robot().getKickerThickness()*0.5f;

    dReal xx = fabs((kx-bx)*vx + (ky-by)*vy);
    dReal yy = fabs(-(kx-bx)*vy + (ky-by)*vx);
    dReal zz = fabs(kz-bz);

    return ((xx < Config::Robot().getKickerThickness() * 2.0f + Config::World().getBallRadius()) && (yy < Config::Robot().getKickerWidth()*0.5f) && (zz < Config::Robot().getKickerHeight() * 0.5f));
}

KickStatus SSLRobot::Kicker::isKicking()
{
    return this->kicking;
}

void SSLRobot::Kicker::setRoller(int roller)
{
    this->rolling = roller;
}

int SSLRobot::Kicker::getRoller()
{
    return this->rolling;
}

void SSLRobot::Kicker::toggleRoller()
{
    if (this->rolling == 0)
        this->rolling = 1;
    else this->rolling = 0;
}

void SSLRobot::Kicker::kick(dReal kickSpeedX, dReal kickSpeedZ)
{    
    dReal dx,dy,dz;
    dReal vx,vy,vz;

    this->rob->chassis->getBodyDirection(dx,dy,dz);
    dz = 0;

    unholdBall();

    if (isTouchingBall())
    {
        dReal dlen = dx*dx + dy*dy + dz*dz; // TODO NAO ENTENDI
        dlen = sqrt(dlen);

        vx = dx*kickSpeedX/dlen;
        vy = dy*kickSpeedX/dlen;
        vz = kickSpeedZ;

        const dReal* vball = dBodyGetLinearVel(rob->getBall()->body);
        dReal vn = -(vball[0]*dx + vball[1]*dy) * Config::Robot().getKickerDampFactor();
        dReal vt = -(vball[0]*dy - vball[1]*dx);
        vx += vn * dx - vt * dy;
        vy += vn * dy + vt * dx;
        dBodySetLinearVel(this->rob->getBall()->body,vx,vy,vz);

        if (kickSpeedZ >= 1)
            this->kicking = CHIP_KICK;
        else
            this->kicking = FLAT_KICK;

        this->kickstate = 10;
    }
}

void SSLRobot::Kicker::holdBall(){
    dReal vx,vy,vz;
    dReal bx,by,bz;
    dReal kx,ky,kz;

    this->rob->chassis->getBodyDirection(vx,vy,vz);
    this->rob->getBall()->getBodyPosition(bx,by,bz);
    this->box->getBodyPosition(kx,ky,kz);

    kx += vx * Config::Robot().getKickerThickness()*0.5f;
    ky += vy * Config::Robot().getKickerThickness()*0.5f;

    dReal xx = fabs((kx-bx)*vx + (ky-by)*vy);
    dReal yy = fabs(-(kx-bx)*vy + (ky-by)*vx);

    if(this->holdingBall || xx - Config::World().getBallRadius() < 0) return;
    dBodySetLinearVel(this->rob->getBall()->body,0,0,0);
    this->robot_to_ball = dJointCreateHinge(this->rob->getWorld()->world, 0);
    dJointAttach(this->robot_to_ball, this->box->body, this->rob->getBall()->body);
    this->holdingBall = true;
}

void SSLRobot::Kicker::unholdBall(){
    if(this->holdingBall) {
        dJointDestroy(this->robot_to_ball);
        this->holdingBall = false;
    }
}

SSLRobot::SSLRobot(PWorld *world, PBall *ball, dReal x, dReal y, dReal z,
               int robot_id, dReal dir)
{
    this->_x = x;
    this->_y = y;
    this->_z = z;
    this->physics = world;
    this->ball = ball;
    this->_dir = dir;
    this->rob_id = robot_id;

    this->space = physics->space;

    this->chassis = new PBox(this->_x, this->_y, this->_z, Config::Robot().getRadius() * 2, Config::Robot().getRadius() * 2, Config::Robot().getHeight(), Config::Robot().getBodyMass() * 0.99f);
    this->chassis->space = this->space;
    this->physics->addObject(chassis);

    this->dummy = new PBall(this->_x, this->_y, this->_z, Config::Robot().getDistanceCenterKicker(), Config::Robot().getBodyMass()*0.01f);
    this->dummy->space = this->space;
    this->physics->addObject(this->dummy);

    this->dummy_to_chassis = dJointCreateFixed(this->physics->world,0);
    dJointAttach(this->dummy_to_chassis, this->chassis->body, this->dummy->body);

    this->kicker = new Kicker(this);

    wheels[0] = new Wheel(this, 0, Config::Robot().getWheel1Angle(), Config::Robot().getWheel1Angle());
    wheels[1] = new Wheel(this, 1, Config::Robot().getWheel2Angle(), Config::Robot().getWheel2Angle());
    wheels[2] = new Wheel(this, 2, Config::Robot().getWheel3Angle(), Config::Robot().getWheel3Angle());
    wheels[3] = new Wheel(this, 3, Config::Robot().getWheel4Angle(), Config::Robot().getWheel4Angle());

    setDir(this->_dir);
}

SSLRobot::~SSLRobot() = default;

PBall *SSLRobot::getBall()
{
    return this->ball;
}

PWorld *SSLRobot::getWorld()
{
    return this->physics;
}

int SSLRobot::getID()
{
    return this->rob_id - 1;
}

void SSLRobot::step()
{
    for (auto &wheel : this->wheels)
        wheel->step();
    this->kicker->step();
}

void SSLRobot::resetSpeeds()
{
    for (auto &wheel : this->wheels)
        wheel->desiredAngularSpeed = 0;
}

void SSLRobot::resetRobot()
{
    resetSpeeds();
    dBodySetLinearVel(this->chassis->body, 0, 0, 0);
    dBodySetAngularVel(this->chassis->body, 0, 0, 0);
    for (auto &wheel : this->wheels)
    {
        dBodySetLinearVel(wheel->cyl->body, 0, 0, 0);
        dBodySetAngularVel(wheel->cyl->body, 0, 0, 0);
    }
    dReal x, y;
    getXY(x, y);
    setXY(x, y);
    setDir(this->_dir);
}

void SSLRobot::getXY(dReal &x, dReal &y)
{
    dReal xx, yy, zz;
    this->chassis->getBodyPosition(xx, yy, zz);
    x = xx;
    y = yy;
}

dReal SSLRobot::getDir()
{
    dReal x, y, z;
    this->chassis->getBodyDirection(x, y, z);

    dReal dot = x; //zarb dar (1.0,0.0,0.0)
    dReal length = sqrt(x * x + y * y);
    auto absAng = (dReal)(acos((dReal)(dot / length)) * (180.0f / M_PI));
    
    return (y > 0) ? absAng : -absAng;
}

dReal SSLRobot::getDir(dReal &k)
{
    dReal x, y, z;
    this->chassis->getBodyDirection(x, y, z, k);

    dReal dot = x; //zarb dar (1.0,0.0,0.0)
    dReal length = sqrt(x * x + y * y);
    auto absAng = (dReal)(acos((dReal)(dot / length)) * (180.0f / M_PI));
    
    return (y > 0) ? absAng : 360-absAng;
}

void SSLRobot::setXY(dReal x, dReal y)
{
    dReal xx, yy, zz, kx, ky, kz;
    dReal height = ROBOT_START_Z();
    this->chassis->getBodyPosition(xx, yy, zz);
    this->chassis->setBodyPosition(x, y, height);
    this->dummy->setBodyPosition(x,y,height);
    this->kicker->box->getBodyPosition(kx,ky,kz);
    this->kicker->box->setBodyPosition(kx-xx+x,ky-yy+y,kz-zz+height);

    for (auto &wheel : this->wheels)
    {
        wheel->cyl->getBodyPosition(kx, ky, kz);
        wheel->cyl->setBodyPosition(kx - xx + x, ky - yy + y, kz - zz + height);
    }
}

void SSLRobot::setDir(dReal ang)
{
    dMatrix3 wLocalRot, wRot, cRot;
    dVector3 localPos, finalPos, cPos;
    ang *= M_PI / 180.0f;

    this->chassis->setBodyRotation(0, 0, 1, ang);
    this->kicker->box->setBodyRotation(0,0,1,ang);
    this->dummy->setBodyRotation(0,0,1,ang);
    
    this->chassis->getBodyPosition(cPos[0], cPos[1], cPos[2], false);
    this->chassis->getBodyRotation(cRot, false);
    dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
    finalPos[0] += cPos[0];
    finalPos[1] += cPos[1];
    finalPos[2] += cPos[2];
    for (auto &wheel : this->wheels)
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

void SSLRobot::setWheelDesiredAngularSpeed(int i, dReal s)
{
    if (!((i >= 4) || (i < 0)))
        this->wheels[i]->desiredAngularSpeed = s;
}

void SSLRobot::setDesiredSpeed(dReal vx, dReal vy, dReal vw)
{
    // Calculate Motor Speeds
    dReal _DEG2RAD = M_PI / 180.0;
    dReal motorAlpha[4] = {Config::Robot().getWheel1Angle() * _DEG2RAD, Config::Robot().getWheel2Angle() * _DEG2RAD, Config::Robot().getWheel3Angle() * _DEG2RAD, Config::Robot().getWheel4Angle() * _DEG2RAD};

    // TODO checar qual a unidade desse valor calculado
    dReal dw1 =  (1.0 / Config::Robot().getWheelRadius()) * (( (Config::Robot().getWheelRadius() * vw) - (vx * sin(motorAlpha[0])) + (vy * cos(motorAlpha[0]))) );
    dReal dw2 =  (1.0 / Config::Robot().getWheelRadius()) * (( (Config::Robot().getWheelRadius() * vw) - (vx * sin(motorAlpha[1])) + (vy * cos(motorAlpha[1]))) );
    dReal dw3 =  (1.0 / Config::Robot().getWheelRadius()) * (( (Config::Robot().getWheelRadius() * vw) - (vx * sin(motorAlpha[2])) + (vy * cos(motorAlpha[2]))) );
    dReal dw4 =  (1.0 / Config::Robot().getWheelRadius()) * (( (Config::Robot().getWheelRadius() * vw) - (vx * sin(motorAlpha[3])) + (vy * cos(motorAlpha[3]))) );

    setWheelDesiredAngularSpeed(0 , dw1);
    setWheelDesiredAngularSpeed(1 , dw2);
    setWheelDesiredAngularSpeed(2 , dw3);
    setWheelDesiredAngularSpeed(3 , dw4);
}