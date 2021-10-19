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

#include "vssworld.h"
#include "vssconfig.h"

#include <cstdlib>
#include <ctime>
#include <utility>
#include <math.h>

#define WHEEL_COUNT 2

VSSWorld *_world;

bool wheelCallBack(dGeomID o1, dGeomID o2, PSurface *surface, int /*robots_count*/)
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
    surface->surface.mu = fric(VSSConfig::Robot().getWheelPerpendicularFriction());
    surface->surface.mu2 = fric(VSSConfig::Robot().getWheelTangentFriction());
    surface->surface.soft_cfm = 0.002;

    dVector3 v = {0, 0, 1, 1};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    surface->fdir1[0] = axis[0];
    surface->fdir1[1] = axis[1];
    surface->fdir1[2] = 0;
    surface->fdir1[3] = 0;
    surface->usefdir1 = true;
    return true;
}

VSSWorld::VSSWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, double timeStep,
             std::vector<double> ballPos, std::vector<std::vector<double>> blueRobotsPos, std::vector<std::vector<double>> yellowRobotsPos)
{
    this->field.setRobotsCount(nRobotsBlue + nRobotsYellow);
    this->field.setRobotsBlueCount(nRobotsBlue);
    this->field.setRobotsYellowCount(nRobotsYellow);
    this->stateSize = 5 + nRobotsBlue * 6 + nRobotsYellow * 6;
    this->state.reserve(this->stateSize);
    this->timeStep = timeStep;
    _world = this;
    this->physics = new PWorld(this->timeStep, 9.81f, this->field.getRobotsCount());
    this->ball = new PBall(ballPos[0], ballPos[1], VSSConfig::World().getBallRadius(), VSSConfig::World().getBallRadius(), VSSConfig::World().getBallMass());
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
        this->robots[k] = new VSSRobot(
            this->physics, this->ball, x, y, ROBOT_START_Z(),
            k + 1, dir, turn_on);
    }
    for (int k = 0; k < this->field.getRobotsYellowCount(); k++)
    {
        bool turn_on = true;
        std::vector<double> robotPos = yellowRobotsPos[k];
        double x = robotPos[0];
        double y = robotPos[1];
        double dir = robotPos[2];
        this->robots[k + this->field.getRobotsBlueCount()] = new VSSRobot(
            this->physics, this->ball, x, y, ROBOT_START_Z(),
            k + 1, dir, turn_on);
    }
    this->physics->initAllObjects();

    //Surfaces
    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1; // | dContactSlip1;
    ballwithwall.surface.mu = 1;                                  //fric(cfg->ballfriction());
    ballwithwall.surface.bounce = VSSConfig::World().getBallBounce();
    ballwithwall.surface.bounce_vel = VSSConfig::World().getBallBounceVel();
    ballwithwall.surface.slip1 = 0; //cfg->ballslip();

    PSurface wheelswithground;
    PSurface *ball_ground = this->physics->createSurface(this->ball, this->ground);
    ball_ground->surface = ballwithwall.surface;

    for (auto &wall : walls)
        this->physics->createSurface(this->ball, wall)->surface = ballwithwall.surface;

    for (int k = 0; k < this->field.getRobotsCount(); k++)
    {
        this->physics->createSurface(robots[k]->chassis, this->ground);
        for (auto &wall : walls)
        {
            this->physics->createSurface(robots[k]->chassis, wall);
        }
        this->physics->createSurface(robots[k]->chassis, this->ball);
        for (auto &wheel : robots[k]->wheels)
        {
            this->physics->createSurface(wheel->cyl, this->ball);
            PSurface *w_g = this->physics->createSurface(wheel->cyl, this->ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (auto &b : robots[k]->balls)
        {
            //            this->physics->createSurface(b->pBall,this->ball);
            PSurface *w_g = this->physics->createSurface(b->pBall, this->ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (int j = k + 1; j < this->field.getRobotsCount(); j++)
        {
            if (k != j)
            {
                this->physics->createSurface(robots[k]->chassis, robots[j]->chassis); //seams ode doesn't understand cylinder-cylinder contacts, so I used spheres
            }
        }
    }

    for (int i = 0; i < 30; i++)
    this->physics->step(this->timeStep * 0.1, this->fullSpeed);
    replace(ballPos, blueRobotsPos, yellowRobotsPos);
}

VSSWorld::~VSSWorld()
{
    for (auto &wall : this->walls) delete(wall);
    for (auto &robot : this->robots) delete(robot);
    delete ball;
    delete ground;
    delete this->physics;
}

void VSSWorld::initWalls()
{
    const double thick = this->field.getWallThickness();
    const double increment = thick / 2; //cfg->Field_Margin() + cfg->Field_Referee_Margin() + thick / 2;
    const double pos_x = this->field.getFieldLength() / 2.0 + increment;
    const double pos_y = this->field.getFieldWidth() / 2.0 + increment;
    const double pos_z = 0.0;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;

    const double gthick = this->field.getWallThickness();
    const double gpos_x = (this->field.getFieldLength() + gthick) / 2.0 + this->field.getGoalDepth();
    const double gpos_y = (this->field.getGoalWidth() + gthick) / 2.0;
    const double gpos_z = 0; //this->field.getGoalHeight() / 2.0;
    const double gsiz_x = this->field.getGoalDepth() + gthick;
    const double gsiz_y = this->field.getGoalWidth();
    const double gsiz_z = siz_z; //this->field.getGoalHeight();
    const double gpos2_x = (this->field.getFieldLength() + gsiz_x) / 2.0;

    this->walls[0] = new PFixedBox(thick / 2, pos_y, pos_z,
                                   siz_x, thick, siz_z);

    this->walls[1] = new PFixedBox(-thick / 2, -pos_y, pos_z,
                                   siz_x, thick, siz_z);

    this->walls[2] = new PFixedBox(pos_x, gpos_y + (siz_y - gsiz_y) / 4, pos_z,
                                   thick, (siz_y - gsiz_y) / 2, siz_z);

    this->walls[10] = new PFixedBox(pos_x, -gpos_y - (siz_y - gsiz_y) / 4, pos_z,
                                    thick, (siz_y - gsiz_y) / 2, siz_z);

    this->walls[3] = new PFixedBox(-pos_x, gpos_y + (siz_y - gsiz_y) / 4, pos_z,
                                   thick, (siz_y - gsiz_y) / 2, siz_z);

    this->walls[11] = new PFixedBox(-pos_x, -gpos_y - (siz_y - gsiz_y) / 4, pos_z,
                                    thick, (siz_y - gsiz_y) / 2, siz_z);

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

    // Corner Wall
    this->walls[12] = new PFixedBox(-pos_x + gsiz_x / 2.8, pos_y - gsiz_x / 2.8, pos_z,
                                    gsiz_x, gthick, gsiz_z);
    this->walls[12]->setRotation(0, 0, 1, M_PI / 4);

    this->walls[13] = new PFixedBox(pos_x - gsiz_x / 2.8, pos_y - gsiz_x / 2.8, pos_z,
                                    gsiz_x, gthick, gsiz_z);
    this->walls[13]->setRotation(0, 0, 1, -M_PI / 4);

    this->walls[14] = new PFixedBox(pos_x - gsiz_x / 2.8, -pos_y + gsiz_x / 2.8, pos_z,
                                    gsiz_x, gthick, gsiz_z);
    this->walls[14]->setRotation(0, 0, 1, M_PI / 4);

    this->walls[15] = new PFixedBox(-pos_x + gsiz_x / 2.8, -pos_y + gsiz_x / 2.8, pos_z,
                                    gsiz_x, gthick, gsiz_z);
    this->walls[15]->setRotation(0, 0, 1, -M_PI / 4);
}

int VSSWorld::robotIndex(unsigned int robot, int team)
{
    if (robot >= this->field.getRobotsCount())
        return -1;
    return robot + team * this->field.getRobotsBlueCount();
}

void VSSWorld::step(std::vector<std::vector<double>> actions)
{

    setActions(std::move(actions));

    for (int k = 0; k < this->field.getRobotsCount(); k++)
    {
        robots[k]->step();
    }

    for (int kk = 0; kk < 5; kk++)
    {
        const dReal *ballvel = dBodyGetLinearVel(this->ball->body);
        // Norma do vetor velocidade da bola
        dReal ballSpeed = ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2];
        ballSpeed = sqrt(ballSpeed);
        if (ballSpeed > 0.01) {
            dReal fk = VSSConfig::World().getBallFriction() * VSSConfig::World().getBallMass() * VSSConfig::World().getGravity();
            dReal ballfx = -fk * ballvel[0] / ballSpeed;
            dReal ballfy = -fk * ballvel[1] / ballSpeed;
            dReal ballfz = -fk * ballvel[2] / ballSpeed;
            dReal balltx = -ballfy * VSSConfig::World().getBallRadius();
            dReal ballty = ballfx * VSSConfig::World().getBallRadius();
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

void VSSWorld::setActions(std::vector<std::vector<double>> actions)
{
    for (int i = 0; i < this->field.getRobotsCount(); i++)
    {
        std::vector<double> rbtAction = actions[i];
        robots[i]->setWheelDesiredAngularSpeed(0, -1 * rbtAction[0]);
        robots[i]->setWheelDesiredAngularSpeed(1, rbtAction[1]);
    }
}

const std::unordered_map<std::string, double> VSSWorld::getFieldParams()
{
    std::unordered_map<std::string, double> fieldParams;

    fieldParams["length"] = this->field.getFieldLength();
    fieldParams["width"] = this->field.getFieldWidth();
    fieldParams["penalty_length"] = this->field.getFieldPenaltyDepth();
    fieldParams["penalty_width"] = this->field.getFieldPenaltyWidth();
    fieldParams["goal_width"] = this->field.getGoalWidth();
    fieldParams["goal_depth"] = this->field.getGoalDepth();
    fieldParams["ball_radius"] = VSSConfig::World().getBallRadius();
    fieldParams["rbt_distance_center_kicker"] = -1.;
    fieldParams["rbt_kicker_thickness"] = -1.;
    fieldParams["rbt_kicker_width"] = -1.;
    fieldParams["rbt_wheel0_angle"] = VSSConfig::Robot().getWheel0Angle();
    fieldParams["rbt_wheel1_angle"] = VSSConfig::Robot().getWheel1Angle();
    fieldParams["rbt_wheel2_angle"] = -1.;
    fieldParams["rbt_wheel3_angle"] = -1.;
    fieldParams["rbt_radius"] = VSSConfig::Robot().getRadius();
    fieldParams["rbt_wheel_radius"] = VSSConfig::Robot().getWheelRadius();
    fieldParams["rbt_motor_max_rpm"] = VSSConfig::Robot().getWheelMotorMaxRPM();
    return fieldParams;
}

const std::vector<double> &VSSWorld::getState()
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
        if (VSSConfig::World().getResetTurnOver() && robotK < 0.9)
        {
            std::cout << "turnover " << robotK << '\n';
            this->robots[i]->resetRobot();
        }

        // Add robot position to state vector
        this->state.push_back(robotX);
        this->state.push_back(robotY);
        this->state.push_back(robotDir);
        if (lastState.size() > 0)
        {
            this->state.push_back((robotX - lastState[5 + (6 * i) + 0]) / this->timeStep);
            this->state.push_back((robotY - lastState[5 + (6 * i) + 1]) / this->timeStep);
            this->state.push_back(smallestAngleDiff(robotDir, lastState[5 + (6 * i) + 2]) / this->timeStep);
        }
        else
        {
            this->state.push_back(0.);
            this->state.push_back(0.);
            this->state.push_back(0.);
        }
    }

    return this->state;
}

void VSSWorld::replace(std::vector<double> ballPos, std::vector<std::vector<double>> blueRobotsPos, std::vector<std::vector<double>> yellowRobotsPos)
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
