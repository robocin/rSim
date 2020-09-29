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

#include "world.h"
#include "config.h"

#include <QtGlobal>
#include <QtNetwork>

#include <cstdlib>
#include <ctime>
#include <math.h>

#define WHEEL_COUNT 2

World *_world;

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
    surface->surface.mu = fric(Config::Robot().getWheelPerpendicularFriction());
    surface->surface.mu2 = fric(Config::Robot().getWheelTangentFriction());
    surface->surface.soft_cfm = 0.002;

    dVector3 v = {0, 0, 1, 1};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    dReal l = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1]);
    surface->fdir1[0] = axis[0] / l;
    surface->fdir1[1] = axis[1] / l;
    surface->fdir1[2] = 0;
    surface->fdir1[3] = 0;
    surface->usefdir1 = true;
    return true;
}

bool ballCallBack(dGeomID o1, dGeomID o2, PSurface *surface, int /*robots_count*/)
{
    if (_world->ball->tag != -1) //spinner adjusting
    {
        dReal x, y, z;
        _world->robots[_world->ball->tag]->chassis->getBodyDirection(x, y, z);
        surface->fdir1[0] = x;
        surface->fdir1[1] = y;
        surface->fdir1[2] = 0;
        surface->fdir1[3] = 0;
        surface->usefdir1 = true;
        surface->surface.mode = dContactMu2 | dContactFDir1 | dContactSoftCFM;
        surface->surface.mu = Config::World().getBallFriction();
        surface->surface.mu2 = 0.5;
        surface->surface.soft_cfm = 0.002;
    }
    return true;
}

World::World(int fieldType, int nRobots)
{
    this->field.setRobotsCount(nRobots);
    this->field.setRobotsBlueCount(nRobots / 2);
    this->field.setRobotsYellowCount(nRobots / 2);
    this->field.setFieldType(fieldType);
    this->episodeSteps = 0;
    this->faultSteps = 0;
    _world = this;
    this->physics = new PWorld(Config::World().getDeltaTime(), 9.81f, this->field.getRobotsCount());
    this->ball = new PBall(0, 0, 0.5, Config::World().getBallRadius(), Config::World().getBallMass());
    this->ground = new PGround(this->field.getFieldRad(), this->field.getFieldLength(), this->field.getFieldWidth(),
                               this->field.getFieldPenaltyDepth(), this->field.getFieldPenaltyWidth(), this->field.getFieldPenaltyPoint(),
                               this->field.getFieldLineWidth(), 0);

    initWalls();

    this->physics->addObject(this->ground);
    this->physics->addObject(this->ball);
    for (auto &wall : this->walls)
        this->physics->addObject(wall);

    // TODO: entender daqui pra baixo
    srand(static_cast<unsigned>(time(0)));
    for (int k = 0; k < this->field.getRobotsCount() * 2; k++)
    {
        bool turn_on = (k % this->field.getRobotsCount() < 3) ? true : false;
        float LO_X = -0.4;
        float LO_Y = -0.3;
        float HI_X = 0.4;
        float HI_Y = 0.3;
        float dir = 1.0;
        float x = LO_X + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_X - LO_X)));
        float y = LO_Y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_Y - LO_Y)));
        x = (k % this->field.getRobotsCount() < 3) ? x : 3.0;
        y = (k % this->field.getRobotsCount() < 3) ? y : 3.0;
        robots[k] = new CRobot(
            this->physics, this->ball, x, y, ROBOT_START_Z(),
            k + 1, dir, turn_on);
    }

    this->physics->initAllObjects();

    //Surfaces
    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1; // | dContactSlip1;
    ballwithwall.surface.mu = 1;                                  //fric(cfg->ballfriction());
    ballwithwall.surface.bounce = Config::World().getBallBounce();
    ballwithwall.surface.bounce_vel = Config::World().getBallBounceVel();
    ballwithwall.surface.slip1 = 0; //cfg->ballslip();

    PSurface wheelswithground;
    PSurface *ball_ground = this->physics->createSurface(this->ball, this->ground);
    ball_ground->surface = ballwithwall.surface;
    ball_ground->callback = ballCallBack;

    for (auto &wall : walls)
        this->physics->createSurface(this->ball, wall)->surface = ballwithwall.surface;

    for (int k = 0; k < 2 * this->field.getRobotsCount(); k++)
    {
        this->physics->createSurface(robots[k]->chassis, this->ground);
        for (auto &wall : walls)
            this->physics->createSurface(robots[k]->chassis, wall);
        this->physics->createSurface(robots[k]->dummy, this->ball);
        //this->physics->createSurface(robots[k]->chassis,this->ball);
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
        for (int j = k + 1; j < 2 * this->field.getRobotsCount(); j++)
        {
            if (k != j)
            {
                this->physics->createSurface(robots[k]->dummy, robots[j]->dummy); //seams ode doesn't understand cylinder-cylinder contacts, so I used spheres
            }
        }
    }
    this->ball->setBodyPosition(0, 0, 0);
    dBodySetLinearVel(this->ball->body, 0, 0, 0);
    dBodySetAngularVel(this->ball->body, 0, 0, 0);
    dReal posX[6] = {0.15, 0.35, 0.71, -0.08, -0.35, -0.71};
    dReal posY[6] = {0.02, 0.13, -0.02, 0.02, 0.13, -0.02};
    for (uint32_t i = 0; i < this->field.getRobotsCount() * 2; i++)
    {
        robots[i]->setXY(posX[i] * (-1), posY[i]);
    }
}

void World::initWalls()
{
    const double thick = this->field.getWallThickness();
    const double increment = thick / 2; //cfg->Field_Margin() + cfg->Field_Referee_Margin() + thick / 2;
    const double pos_x = this->field.getFieldLength() / 2.0 + increment;
    const double pos_y = this->field.getFieldWidth() / 2.0 + increment;
    const double pos_z = 0.0;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;
    const double tone = 1.0;

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

int World::robotIndex(unsigned int robot, int team)
{
    if (robot >= this->field.getRobotsCount())
        return -1;
    return robot + team * this->field.getRobotsCount();
}

World::~World()
{
    delete this->physics;
}

void World::step(dReal dt, std::vector<std::tuple<double, double>> actions)
{
    setActions(actions);

    // Pq ele faz isso 5 vezes?
    // - Talvez mais precisao (Ele sempre faz um step de dt*0.2 )
    for (int kk = 0; kk < 5; kk++)
    {
        const dReal *ballvel = dBodyGetLinearVel(this->ball->body);
        // Norma do vetor velocidade da bola
        dReal ballspeed = ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2];
        ballspeed = sqrt(ballspeed);
        dReal ballfx = 0, ballfy = 0, ballfz = 0;
        dReal balltx = 0, ballty = 0, balltz = 0;
        if (ballspeed < 0.01)
        {
            ; //const dReal* ballAngVel = dBodyGetAngularVel(this->ball->body);
            //TODO: what was supposed to be here?
        }
        else
        {
            // Velocidade real  normalizada (com atrito envolvido) da bola
            dReal accel = last_speed - ballspeed;
            accel = -accel / dt;
            last_speed = ballspeed;
            dReal fk = accel * Config::World().getBallFriction() * Config::World().getBallMass() * Config::World().getGravity();
            ballfx = -fk * ballvel[0] / ballspeed;
            ballfy = -fk * ballvel[1] / ballspeed;
            ballfz = -fk * ballvel[2] / ballspeed;
            balltx = -ballfy * Config::World().getBallRadius();
            ballty = ballfx * Config::World().getBallRadius();
            balltz = 0;
            dBodyAddTorque(this->ball->body, balltx, ballty, balltz);
        }
        dBodyAddForce(this->ball->body, ballfx, ballfy, ballfz);

        this->physics->step(dt * 0.2, fullSpeed);
    }

    for (int k = 0; k < this->field.getRobotsCount() * 2; k++)
    {
        robots[k]->step();
    }
    this->episodeSteps++;
}

void World::setActions(std::vector<std::tuple<double, double>> actions)
{
    int id = 0;
    for (std::tuple<double, double> robotAction : actions)
    {
        robots[id]->setSpeed(0, -1 * std::get<0>(robotAction));
        robots[id]->setSpeed(1, std::get<1>(robotAction));
        id++;
    }
}

int World::getEpisodeTime()
{
    return this->episodeSteps * Config::World().getDeltaTime() * 1000;
}

const std::vector<int> World::getGoals()
{
    std::vector<int> goal = std::vector<int>(static_cast<std::size_t>(2));
    goal.clear();
    goal.push_back(this->goalsBlue);
    goal.push_back(this->goalsYellow);
    return goal;
}

const std::vector<double> World::getFieldParams()
{
    std::vector<double> field = std::vector<double>(static_cast<std::size_t>(6));
    field.clear();
    field.push_back(this->field.getFieldWidth());
    field.push_back(this->field.getFieldLength());
    field.push_back(this->field.getFieldPenaltyWidth());
    field.push_back(this->field.getFieldPenaltyDepth());
    field.push_back(this->field.getGoalWidth());
    field.push_back(this->field.getGoalDepth());
    return field;
}

const std::vector<double> &World::getState()
{
    this->state.clear();
    dReal ballX, ballY, ballZ;
    dReal robotX, robotY, robotDir, robotK;
    const dReal *ballVel, *robotVel;

    // Set noise parameters
    dReal devX = 0;
    dReal devY = 0;
    dReal devA = 0;
    if (Config::Noise().getNoise())
    {
        devX = Config::Noise().getNoiseDeviationX();
        devY = Config::Noise().getNoiseDeviationY();
        devA = Config::Noise().getNoiseDeviationAngle();
    }

    // Ball
    this->ball->getBodyPosition(ballX, ballY, ballZ);
    ballVel = dBodyGetLinearVel(this->ball->body);

    // Add ball position to state vector
    this->state.push_back(randn_notrig(ballX, devX));
    this->state.push_back(randn_notrig(ballY, devY));
    this->state.push_back(ballZ);
    this->state.push_back(ballVel[0]);
    this->state.push_back(ballVel[1]);

    // Robots
    for (uint32_t i = 0; i < this->field.getRobotsCount() * 2; i++)
    {
        this->robots[i]->getXY(robotX, robotY);
        // robotDir is not currently being used
        robotDir = this->robots[i]->getDir(robotK);
        robotVel = dBodyGetLinearVel(this->robots[i]->chassis->body);

        // reset when the robot has turned over
        if (Config::World().getResetTurnOver() && robotK < 0.9)
        {
            this->robots[i]->resetRobot();
        }

        // Add robot position to state vector
        this->state.push_back(randn_notrig(robotX, devX));
        this->state.push_back(randn_notrig(robotY, devY));
        this->state.push_back(robotVel[0]);
        this->state.push_back(robotVel[1]);
    }
    return this->state;
}

void World::replace(double *ball, double *pos_blue, double *pos_yellow)
{
    this->ball->setBodyPosition(ball[0], ball[1], 0);
    dBodySetLinearVel(this->ball->body, 0, 0, 0);
    dBodySetAngularVel(this->ball->body, 0, 0, 0);
    std::vector<std::vector<double>> blues;
    blues.clear();
    for (int i = 0; i < this->field.getRobotsBlueCount(); i = i + 2)
    {
        std::vector<double> pos;
        pos.clear();
        pos.push_back(pos_blue[i]);
        pos.push_back(pos_blue[i + 1]);
        blues.push_back(pos);
    }

    std::vector<std::vector<double>> yellows;
    yellows.clear();
    for (int i = 0; i < this->field.getRobotsYellowCount(); i = i + 2)
    {
        std::vector<double> pos;
        pos.clear();
        pos.push_back(pos_yellow[i]);
        pos.push_back(pos_yellow[i + 1]);
        yellows.push_back(pos);
    }

    for (uint32_t i = 0; i < this->field.getRobotsBlueCount(); i++)
    {
        robots[i]->setXY(blues[i][0] * (-1), blues[i][1]);
    }
    for (uint32_t i = this->field.getRobotsBlueCount(); i < this->field.getRobotsYellowCount(); i++)
    {
        uint32_t k = i - this->field.getRobotsBlueCount();
        robots[i]->setXY(yellows[k][0] * (-1), yellows[k][1]);
    }
}

void World::getValidPosition(dReal &x, dReal &y, uint32_t max)
{
    float LO_X = -0.65;
    float LO_Y = -0.55;
    float HI_X = 0.65;
    float HI_Y = 0.55;
    srand(static_cast<unsigned>(time(0)));
    bool validPlace;
    max = max > 0 ? max : this->field.getRobotsCount() * 2;
    do
    {
        validPlace = true;
        x = LO_X + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_X - LO_X)));
        y = LO_Y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_Y - LO_Y)));
        for (uint32_t i = 0; i < max; i++)
        {
            dReal x2, y2;
            robots[i]->getXY(x2, y2);
            if (sqrt(((x - x2) * (x - x2)) + ((y - y2) * (y - y2))) <= (Config::Robot().getRadius() * 2))
            {
                validPlace = false;
            }
        }
    } while (!validPlace);
}
