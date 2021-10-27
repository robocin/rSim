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

#include "sslworld.h"
#include "sslconfig.h"

#include <cstdlib>
#include <ctime>
#include <math.h>


bool sslWheelCallBack(dGeomID o1, dGeomID o2, PSurface *surface, int /*robots_count*/)
{
    //s->id2 is ground
    const dReal *r; //wheels rotation matrix
    //const dReal* p; //wheels rotation matrix
    if ((o1 == surface->id1) && (o2 == surface->id2))
    {
        r = dBodyGetRotation(dGeomGetBody(o1));
        //p=dGeomGetPosition(o1);//never read
    }
    else if ((o1 == surface->id2) && (o2 == surface->id1))
    {
        r = dBodyGetRotation(dGeomGetBody(o2));
        //p=dGeomGetPosition(o2);//never read
    }
    else
    {
        //XXX: in this case we dont have the rotation
        //     matrix, thus we must return
        return false;
    }

    surface->surface.mode = dContactFDir1 | dContactMu2 | dContactApprox1 | dContactSoftCFM;
    surface->surface.mu = fric(SSLConfig::Robot().getWheelPerpendicularFriction());
    surface->surface.mu2 = fric(SSLConfig::Robot().getWheelTangentFriction());
    surface->surface.soft_cfm = 0.002;

    dVector3 v = {0, 0, 1, 1};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    // dReal l = sqrt(axis[0] * axis[0] + axis[1] * axis[1]);
    surface->fdir1[0] = axis[0];
    surface->fdir1[1] = axis[1];
    surface->fdir1[2] = 0;
    surface->fdir1[3] = 0;
    surface->usefdir1 = true;
    return true;
}

bool sslBallCallBack(dGeomID o1, dGeomID o2, PSurface *surface, int /*robots_count*/)
{
    auto body = dGeomGetBody(o2);
    const dReal *posBall = dBodyGetPosition(body);
    body = dGeomGetBody(o1);
    const dReal *posRobot = dBodyGetPosition(body);
    const dReal *dirRobot = dBodyGetRotation(body);

    dReal distRbtBall = sqrt(pow(posRobot[0]-posBall[0],2) + pow(posRobot[1]-posBall[1],2));
    if (distRbtBall < SSLConfig::Robot().getDistanceCenterKicker() + SSLConfig::World().getBallRadius()/2.) return true; 

    // Get robot angle
    dVector3 v={1,0,0};
    dVector3 axis;
    dMultiply0(axis,dirRobot,v,4,3,1);
    dReal dot = axis[0];
    dReal length = sqrt(axis[0]*axis[0] + axis[1]*axis[1]);
    dReal absAng = (dReal)(acos((dReal)(dot/length)));
    dReal angleRobot =  (axis[1] > 0) ? absAng : -absAng;

    // Get angle between robot and ball
    dReal angleRobotBall = atan2(posBall[1] - posRobot[1],posBall[0]-posRobot[0]);

    // This value is given by the acos(distance_center_kicker/robot_radius)
    dReal angleKicker = SSLConfig::Robot().getKickerAngle();

    // Smallest angle diff
    dReal angleDiff = angleRobotBall - angleRobot;
    angleDiff += (angleDiff>M_PI) ? -2*M_PI : (angleDiff<-M_PI) ? 2*M_PI : 0;

    // If kicker is facing the ball, the collision with the chassis should not
    // be considered
    return (abs(angleDiff) < angleKicker) ? false : true;
}

SSLWorld::SSLWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, double timeStep,
                   std::vector<double> ballPos, std::vector<std::vector<double>> blueRobotsPos, std::vector<std::vector<double>> yellowRobotsPos)
{
    // fieldType = 0 for Div A, fieldType = 1 for Div B
    this->field.setFieldType(fieldType);
    this->field.setRobotsCount(nRobotsBlue + nRobotsYellow);
    this->field.setRobotsBlueCount(nRobotsBlue);
    this->field.setRobotsYellowCount(nRobotsYellow);
    this->stateSize = 5 + nRobotsBlue * 7 + nRobotsYellow * 7;
    this->state.reserve(this->stateSize);
    this->timeStep = timeStep;
    this->physics = new PWorld(this->timeStep, 9.81f, this->field.getRobotsCount());
    this->ball = new PBall(ballPos[0], ballPos[1], SSLConfig::World().getBallRadius(), SSLConfig::World().getBallRadius(), SSLConfig::World().getBallMass());
    this->ground = new PGround(this->field.getFieldRad(), this->field.getFieldLength(), this->field.getFieldWidth(),
                               this->field.getFieldPenaltyDepth(), this->field.getFieldPenaltyWidth(), this->field.getFieldPenaltyPoint(),
                               this->field.getFieldLineWidth());

    initWalls();

    this->physics->addGroundObject(this->ground);
    this->physics->addBallObject(this->ball);

    for (auto &wall : this->walls)
        this->physics->addWallObject(wall);

    for (int k = 0; k < this->field.getRobotsBlueCount(); k++)
    {
        bool turn_on = true;
        std::vector<double> robotPos = blueRobotsPos[k];
        double x = robotPos[0];
        double y = robotPos[1];
        double dir = robotPos[2];
        this->robots[k] = new SSLRobot(
            this->physics, this->ball, x, y, SSL_ROBOT_START_Z(),
            k + 1, dir);
    }
    for (int k = 0; k < this->field.getRobotsYellowCount(); k++)
    {
        bool turn_on = true;
        std::vector<double> robotPos = yellowRobotsPos[k];
        double x = robotPos[0];
        double y = robotPos[1];
        double dir = robotPos[2];
        robots[k + this->field.getRobotsBlueCount()] = new SSLRobot(
            this->physics, this->ball, x, y, SSL_ROBOT_START_Z(),
            k + 1, dir);
    }

    this->physics->initAllObjects();

    //Surfaces
    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1; // | dContactSlip1;
    ballwithwall.surface.mu = 1;                                  //fric(cfg->ballfriction());
    ballwithwall.surface.bounce = SSLConfig::World().getBallBounce();
    ballwithwall.surface.bounce_vel = SSLConfig::World().getBallBounceVel();
    ballwithwall.surface.slip1 = 0; //cfg->ballslip();

    PSurface wheelswithground;
    PSurface *ball_ground = this->physics->createOneWaySurface(this->ball, this->ground);
    ball_ground->surface = ballwithwall.surface;

    PSurface ballwithkicker;
    ballwithkicker.surface.mode = dContactApprox1;
    ballwithkicker.surface.mu = fric(SSLConfig::Robot().getKickerFriction());
    ballwithkicker.surface.slip1 = 5;

    for (auto &wall : walls)
        this->physics->createOneWaySurface(this->ball, wall)->surface = ballwithwall.surface;

    for (int k = 0; k < this->field.getRobotsCount(); k++)
    {
        for (auto &wall : walls)
        {
            this->physics->createOneWaySurface(this->robots[k]->chassis, wall);
        }

        // Create surface between ball and chassis
        PSurface* ballChassis = this->physics->createOneWaySurface(this->robots[k]->chassis, this->ball);
        ballChassis->callback = sslBallCallBack;

        this->physics->createOneWaySurface(this->ball, this->robots[k]->kicker->box)->surface = ballwithkicker.surface;
        for (auto &wheel : this->robots[k]->wheels)
        {
            PSurface *w_g = this->physics->createOneWaySurface(wheel->cyl, this->ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = sslWheelCallBack;
        }
        for (int j = k + 1; j < this->field.getRobotsCount(); j++)
        {
            if (k != j)
            {
                this->physics->createSurface(this->robots[k]->chassis, this->robots[j]->chassis);
            }
        }
    }

    for (int i = 0; i < 30; i++)
    this->physics->step(this->timeStep * 0.1, this->fullSpeed);
    replace(ballPos, blueRobotsPos, yellowRobotsPos);
}

SSLWorld::~SSLWorld()
{
    for (auto &wall : this->walls) delete(wall);
    for (auto &robot : this->robots) delete(robot);
    delete ball;
    delete ground;
    delete this->physics;
}

void SSLWorld::initWalls()
{
    const double thick = this->field.getWallThickness();
    const double increment = this->field.getFieldMargin() + this->field.getFieldRefereeMargin() + thick / 2;
    const double pos_x = this->field.getFieldLength() / 2.0 + increment;
    const double pos_y = this->field.getFieldWidth() / 2.0 + increment;
    const double pos_z = 0.0;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;

    const double gthick = this->field.getGoalThickness();
    const double gpos_x = (this->field.getFieldLength() + gthick) / 2.0 + this->field.getGoalDepth();
    const double gpos_y = (this->field.getGoalWidth() + gthick) / 2.0;
    const double gpos_z = this->field.getGoalHeight() / 2.0;
    const double gsiz_x = this->field.getGoalDepth() + gthick;
    const double gsiz_y = this->field.getGoalWidth();
    const double gsiz_z = this->field.getGoalHeight();
    const double gpos2_x = (this->field.getFieldLength() + gsiz_x) / 2.0;

    this->walls[0] = new PFixedBox(thick/2, pos_y, pos_z,
                             siz_x, thick, siz_z);

    this->walls[1] = new PFixedBox(-thick/2, -pos_y, pos_z,
                             siz_x, thick, siz_z);
    
    this->walls[2] = new PFixedBox(pos_x, -thick/2, pos_z,
                             thick, siz_y, siz_z);

    this->walls[3] = new PFixedBox(-pos_x, thick/2, pos_z,
                             thick, siz_y, siz_z);

    // Goal walls
    
    this->walls[4] = new PFixedBox(gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z);
    
    this->walls[5] = new PFixedBox(gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);
    
    this->walls[6] = new PFixedBox(gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);

    this->walls[7] = new PFixedBox(-gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z);
    
    this->walls[8] = new PFixedBox(-gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);
    
    this->walls[9] = new PFixedBox(-gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);
}


void SSLWorld::step(std::vector<std::vector<double>> actions)
{

    setActions(std::move(actions));

    for (int k = 0; k < this->field.getRobotsCount(); k++)
    {
        robots[k]->step();
    }

    // Pq ele faz isso 5 vezes?
    // - Talvez mais precisao (Ele sempre faz um step de dt*0.2 )
    for (int kk = 0; kk < 5; kk++)
    {
        const dReal *ballvel = dBodyGetLinearVel(this->ball->body);
        // Norma do vetor velocidade da bola
        dReal ballSpeed = ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2];
        ballSpeed = sqrt(ballSpeed);
        if (ballSpeed > 0.01) {
            dReal fk = SSLConfig::World().getBallFriction() * SSLConfig::World().getBallMass() * SSLConfig::World().getGravity();
            dReal ballfx = -fk * ballvel[0] / ballSpeed;
            dReal ballfy = -fk * ballvel[1] / ballSpeed;
            dReal ballfz = -fk * ballvel[2] / ballSpeed;
            dReal balltx = -ballfy * SSLConfig::World().getBallRadius();
            dReal ballty = ballfx * SSLConfig::World().getBallRadius();
            dReal balltz = 0;
            dBodyAddTorque(this->ball->body,balltx,ballty,balltz);
            dBodyAddForce(this->ball->body,ballfx,ballfy,ballfz);
        } else {
            dBodySetAngularVel(this->ball->body, 0, 0, 0);
            dBodySetLinearVel(this->ball->body, 0, 0, 0);
        }

        this->physics->step(this->timeStep * 0.2, this->fullSpeed);
    }

}

void SSLWorld::setActions(std::vector<std::vector<double>> actions)
{
    for (int i = 0; i < this->field.getRobotsCount(); i++)
    {
        std::vector<double> rbtAction = actions[i];
        if (rbtAction[0] > 0) {
            this->robots[i]->setWheelDesiredAngularSpeed(0, rbtAction[1]);
            this->robots[i]->setWheelDesiredAngularSpeed(1, rbtAction[2]);
            this->robots[i]->setWheelDesiredAngularSpeed(2, rbtAction[3]);
            this->robots[i]->setWheelDesiredAngularSpeed(3, rbtAction[4]);
        } 
        else this->robots[i]->setDesiredSpeedLocal(rbtAction[1], rbtAction[2], rbtAction[3]);
        if (rbtAction[5] > 0 || rbtAction[6] > 0) {
            this->robots[i]->kicker->kick(rbtAction[5], rbtAction[6]);
        }
        if (rbtAction[7] > 0) this->robots[i]->kicker->setDribbler(true);
    }
}

const std::unordered_map<std::string, double> SSLWorld::getFieldParams()
{
    std::unordered_map<std::string, double> fieldParams;

    fieldParams["length"] = this->field.getFieldLength();
    fieldParams["width"] = this->field.getFieldWidth();
    fieldParams["penalty_length"] = this->field.getFieldPenaltyDepth();
    fieldParams["penalty_width"] = this->field.getFieldPenaltyWidth();
    fieldParams["goal_width"] = this->field.getGoalWidth();
    fieldParams["goal_depth"] = this->field.getGoalDepth();
    fieldParams["ball_radius"] = SSLConfig::World().getBallRadius();
    fieldParams["rbt_distance_center_kicker"] = SSLConfig::Robot().getDistanceCenterKicker();
    fieldParams["rbt_kicker_thickness"] = SSLConfig::Robot().getKickerThickness();
    fieldParams["rbt_kicker_width"] = SSLConfig::Robot().getKickerWidth();
    fieldParams["rbt_wheel0_angle"] = SSLConfig::Robot().getWheel0Angle();
    fieldParams["rbt_wheel1_angle"] = SSLConfig::Robot().getWheel1Angle();
    fieldParams["rbt_wheel2_angle"] = SSLConfig::Robot().getWheel2Angle();
    fieldParams["rbt_wheel3_angle"] = SSLConfig::Robot().getWheel3Angle();
    fieldParams["rbt_radius"] = SSLConfig::Robot().getRadius();
    fieldParams["rbt_wheel_radius"] = SSLConfig::Robot().getWheelRadius();
    fieldParams["rbt_motor_max_rpm"] = SSLConfig::Robot().getWheelMotorMaxRPM();

    return fieldParams;
}

const std::vector<double> &SSLWorld::getState()
{
    std::vector<double> lastState;
    lastState = this->state;
    this->state.clear();

    dReal ballX, ballY, ballZ;
    dReal robotX, robotY, robotDir, robotK;
    const dReal *ballVel, *robotVel, *robotVelDir;

    // Ball
    this->ball->getBodyPosition(ballX, ballY, ballZ);

    // Add ball position to state vector
    this->state.push_back(ballX);
    this->state.push_back(ballY);
    this->state.push_back(ballZ);
    if (lastState.size() > 0)
    {
        this->state.push_back((ballX - lastState[0]) / this->timeStep);
        this->state.push_back((ballY - lastState[1]) / this->timeStep);
    }
    else
    {
        ballVel = dBodyGetLinearVel(this->ball->body);
        this->state.push_back(0.);
        this->state.push_back(0.);
    }

    // Robots
    for (uint32_t i = 0; i < this->field.getRobotsCount(); i++)
    {
        this->robots[i]->getXY(robotX, robotY);

        // robotDir is not currently being used
        robotDir = this->robots[i]->getDir(robotK);
        robotVel = dBodyGetLinearVel(this->robots[i]->chassis->body);
        robotVelDir = dBodyGetAngularVel(this->robots[i]->chassis->body);
        // reset when the robot has turned over
        if (SSLConfig::World().getResetTurnOver() && robotK < 0.9)
        {
            std::cout << "turnover " << robotK << " robot x: " << robotX << " robot y: " << robotY << " robot dir: " << robotDir << " ball x: " << ballX  << " ball y: " << ballY << '\n';
            this->robots[i]->resetRobot();
        }

        // Add robot position to state vector
        this->state.push_back(robotX);
        this->state.push_back(robotY);
        this->state.push_back(robotDir);
        if (lastState.size() > 0)
        {
            this->state.push_back((robotX - lastState[5 + (11 * i) + 0]) / this->timeStep);
            this->state.push_back((robotY - lastState[5 + (11 * i) + 1]) / this->timeStep);
            this->state.push_back(smallestAngleDiff(robotDir, lastState[5 + (11 * i) + 2]) / this->timeStep);
        }
        else
        {
            this->state.push_back(0.);
            this->state.push_back(0.);
            this->state.push_back(0.);
        }
        this->state.push_back(static_cast<double>(this->robots[i]->kicker->isTouchingBall()));

        // Get wheel speeds
        this->state.push_back(this->robots[i]->wheels[0]->desiredAngularSpeed);
        this->state.push_back(this->robots[i]->wheels[1]->desiredAngularSpeed);
        this->state.push_back(this->robots[i]->wheels[2]->desiredAngularSpeed);
        this->state.push_back(this->robots[i]->wheels[3]->desiredAngularSpeed);
    }

    return this->state;
}

void SSLWorld::replace(std::vector<double> ballPos, std::vector<std::vector<double>> blueRobotsPos, std::vector<std::vector<double>> yellowRobotsPos)
{
    dReal xx, yy, zz;
    this->ball->getBodyPosition(xx, yy, zz);
    this->ball->setBodyPosition(ballPos[0], ballPos[1], zz);
    dBodySetLinearVel(this->ball->body, ballPos[2], ballPos[3], 0);
    dBodySetAngularVel(this->ball->body, 0, 0, 0);

    for (uint32_t i = 0; i < this->field.getRobotsBlueCount(); i++)
    {
        std::vector<double> robotPos = blueRobotsPos[i];
        this->robots[i]->resetRobot();
        this->robots[i]->setXY(robotPos[0], robotPos[1]);
        this->robots[i]->setDir(robotPos[2]);
    }

    for (int32_t i  = 0; i < this->field.getRobotsYellowCount(); i++)
    {
        int k = i + this->field.getRobotsBlueCount();
        std::vector<double> robotPos = yellowRobotsPos[i];
        this->robots[k]->resetRobot();
        this->robots[k]->setXY(robotPos[0], robotPos[1]);
        this->robots[k]->setDir(robotPos[2]);
    }
}
