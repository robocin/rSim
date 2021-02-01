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

World::World(int fieldType, int nRobotsBlue, int nRobotsYellow, double timeStep,
             double *ballPos, double *blueRobotsPos, double *yellowRobotsPos)
{
    this->field.setRobotsCount(nRobotsBlue + nRobotsYellow);
    this->field.setRobotsBlueCount(nRobotsBlue);
    this->field.setRobotsYellowCount(nRobotsYellow);
    this->field.setFieldType(fieldType);
    this->timeStep = timeStep;
    this->episodeSteps = 0;
    _world = this;
    this->physics = new PWorld(this->timeStep, 9.81f, this->field.getRobotsCount());
    this->ball = new PBall(ballPos[0], ballPos[1], Config::World().getBallRadius(), Config::World().getBallRadius(), Config::World().getBallMass());
    this->ground = new PGround(this->field.getFieldRad(), this->field.getFieldLength(), this->field.getFieldWidth(),
                               this->field.getFieldPenaltyDepth(), this->field.getFieldPenaltyWidth(), this->field.getFieldPenaltyPoint(),
                               this->field.getFieldLineWidth());

    initWalls();

    this->physics->addObject(this->ground);
    this->physics->addObject(this->ball);

    for (auto &wall : this->walls)
        this->physics->addObject(wall);

    // TODO: entender daqui pra baixo
    for (int k = 0; k < this->field.getRobotsBlueCount(); k++)
    {
        bool turn_on = true;
        float x = blueRobotsPos[k * 3];
        float y = blueRobotsPos[(k * 3) + 1];
        float dir = blueRobotsPos[(k * 3) + 2];
        this->robots[k] = new SSLRobot(
            this->physics, this->ball, x, y, ROBOT_START_Z(),
            k + 1, dir);
    }
    for (int k = this->field.getRobotsBlueCount(); k < this->field.getRobotsCount(); k++)
    {
        bool turn_on = true;
        int i = k - this->field.getRobotsBlueCount();
        float x = yellowRobotsPos[i * 3];
        float y = yellowRobotsPos[(i * 3) + 1];
        float dir = yellowRobotsPos[(i * 3) + 2];
        this->robots[k] = new SSLRobot(
            this->physics, this->ball, x, y, ROBOT_START_Z(),
            k + 1, dir);
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

    PSurface ballwithkicker;
    ballwithkicker.surface.mode = dContactApprox1;
    ballwithkicker.surface.mu = fric(Config::Robot().getKickerFriction());
    ballwithkicker.surface.slip1 = 5;

    for (auto &wall : walls)
        this->physics->createSurface(this->ball, wall)->surface = ballwithwall.surface;

    for (int k = 0; k < this->field.getRobotsCount(); k++)
    {
        this->physics->createSurface(this->robots[k]->chassis, this->ground);
        for (auto &wall : walls)
        {
            this->physics->createSurface(this->robots[k]->chassis, wall);
        }
        this->physics->createSurface(this->robots[k]->dummy, this->ball);
        this->physics->createSurface(this->robots[k]->kicker->box, this->ball)->surface = ballwithkicker.surface;
        for (auto &wheel : this->robots[k]->wheels)
        {
            this->physics->createSurface(wheel->cyl, this->ball);
            PSurface *w_g = this->physics->createSurface(wheel->cyl, this->ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (int j = k + 1; j < this->field.getRobotsCount(); j++)
        {
            if (k != j)
            {
                this->physics->createSurface(this->robots[k]->dummy, this->robots[j]->dummy); //seams ode doesn't understand cylinder-cylinder contacts, so I used spheres
                this->physics->createSurface(this->robots[k]->chassis, this->robots[j]->kicker->box);
            }
        }
    }
}

void World::initWalls()
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


World::~World()
{
    delete this->physics;
}

void World::step(dReal dt, std::vector<std::tuple<double, double, double, double, bool, double, double, bool>> actions)
{
    setActions(actions);

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
        dReal ballfx = 0, ballfy = 0, ballfz = 0;
        dReal balltx = 0, ballty = 0, balltz = 0;
        if (ballSpeed > 0.01)
        {
            dReal fk = Config::World().getBallFriction() * Config::World().getBallMass() * Config::World().getGravity();
            ballfx = -fk * ballvel[0] / ballSpeed;
            ballfy = -fk * ballvel[1] / ballSpeed;
            ballfz = -fk * ballvel[2] / ballSpeed;
            balltx = -ballfy * Config::World().getBallRadius();
            ballty = ballfx * Config::World().getBallRadius();
            balltz = 0;
            dBodyAddTorque(this->ball->body, balltx, ballty, balltz);
        }
        dBodyAddForce(this->ball->body, ballfx, ballfy, ballfz);

        this->physics->step(dt * 0.2, this->fullSpeed);
    }

    this->episodeSteps++;
}

// TODO : Decidir se vai implementar o RobotStatus

// TODO : Vector de tuples é o melhor formato?
// TODO : Precisa dividir em outra função para vx, vy e vtheta e coord locais e globais
void World::setActions(std::vector<std::tuple<double, double, double, double, bool, double, double, bool>> actions)
{
    int id = 0;
    for (int i = 0; i < this->field.getRobotsBlueCount(); i++)
    {
        this->robots[i]->setWheelDesiredAngularSpeed(0, std::get<0>(actions[i]));
        this->robots[i]->setWheelDesiredAngularSpeed(1, std::get<1>(actions[i]));
        this->robots[i]->setWheelDesiredAngularSpeed(2, std::get<2>(actions[i]));
        this->robots[i]->setWheelDesiredAngularSpeed(3, std::get<3>(actions[i]));
        if (std::get<4>(actions[i])) {
            this->robots[i]->kicker->kick(std::get<5>(actions[i]), std::get<6>(actions[i]));
        }
        this->robots[i]->kicker->setRoller(std::get<7>(actions[i]));
    }
    for (int i = this->field.getRobotsBlueCount(); i < this->field.getRobotsCount(); i++)
    {
        this->robots[i]->setWheelDesiredAngularSpeed(0, std::get<0>(actions[i]));
        this->robots[i]->setWheelDesiredAngularSpeed(1, std::get<1>(actions[i]));
        this->robots[i]->setWheelDesiredAngularSpeed(2, std::get<2>(actions[i]));
        this->robots[i]->setWheelDesiredAngularSpeed(3, std::get<3>(actions[i]));
        if (std::get<4>(actions[i])) {
            this->robots[i]->kicker->kick(std::get<5>(actions[i]), std::get<6>(actions[i]));
        }
        this->robots[i]->kicker->setRoller(std::get<7>(actions[i]));
    }
}

// TODO : NÃO ESTA SENDO USADO ATUALMENTE
int World::getEpisodeTime()
{
    return this->episodeSteps * static_cast<int>(getTimeStep() * 1000);
}

// TODO : NÃO ESTA SENDO USADO ATUALMENTE
const std::vector<int> World::getGoals()
{
    std::vector<int> goal = std::vector<int>(static_cast<std::size_t>(2));
    goal.clear();
    goal.push_back(this->goalsBlue);
    goal.push_back(this->goalsYellow);
    return goal;
}

// TODO : ESTA SENDO UTILIZADO?
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
    std::vector<double> last_state = this->state;

    this->state.clear();
    dReal ballX, ballY, ballZ;
    dReal robotX, robotY, robotDir, robotK;
    const dReal *ballVel, *robotVel, *robotVelDir;

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

    // Add ball position to state vector
    this->state.push_back(randn_notrig(ballX, devX));
    this->state.push_back(randn_notrig(ballY, devY));
    this->state.push_back(ballZ);
    if (last_state.size() > 0)
    {
        this->state.push_back((ballX - last_state[0]) / this->timeStep);
        this->state.push_back((ballY - last_state[1]) / this->timeStep);
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
        if (Config::World().getResetTurnOver() && robotK < 0.9)
        {
            std::cout << "turnover " << robotK << '\n';
            this->robots[i]->resetRobot();
        }

        // Add robot position to state vector
        this->state.push_back(robotX);
        this->state.push_back(robotY);
        this->state.push_back(robotDir);
        if (last_state.size() > 0)
        {
            this->state.push_back((robotX - last_state[5 + (6 * i) + 0]) / this->timeStep);
            this->state.push_back((robotY - last_state[5 + (6 * i) + 1]) / this->timeStep);
            this->state.push_back(smallestAngleDiff(robotDir, last_state[5 + (6 * i) + 2]) / this->timeStep);
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

void World::replace(double *ball, double *pos_blue, double *pos_yellow)
{
    this->ball->setBodyPosition(ball[0], ball[1], 0);
    dBodySetLinearVel(this->ball->body, 0, 0, 0);
    dBodySetAngularVel(this->ball->body, 0, 0, 0);
    std::vector<std::vector<double>> blues;
    blues.clear();
    for (int i = 0; i < this->field.getRobotsBlueCount() * 3; i = i + 3)
    {
        std::vector<double> pos;
        pos.clear();
        pos.push_back(pos_blue[i]);
        pos.push_back(pos_blue[i + 1]);
        pos.push_back(pos_blue[i + 2]);
        blues.push_back(pos);
    }

    std::vector<std::vector<double>> yellows;
    yellows.clear();
    for (int i = 0; i < this->field.getRobotsYellowCount() * 3; i = i + 3)
    {
        std::vector<double> pos;
        pos.clear();
        pos.push_back(pos_yellow[i]);
        pos.push_back(pos_yellow[i + 1]);
        pos.push_back(pos_yellow[i + 2]);
        yellows.push_back(pos);
    }

    for (uint32_t i = 0; i < this->field.getRobotsBlueCount(); i++)
    {
        this->robots[i]->setXY(blues[i][0], blues[i][1]);
        this->robots[i]->setDir(blues[i][2]);
    }
    for (uint32_t i = this->field.getRobotsBlueCount(); i < this->field.getRobotsYellowCount() + this->field.getRobotsBlueCount(); i++)
    {
        uint32_t k = i - this->field.getRobotsBlueCount();
        this->robots[i]->setXY(yellows[k][0], yellows[k][1]);
        this->robots[i]->setDir(yellows[k][2]);
    }
}

void World::replace_with_vel(double *ball, double *pos_blue, double *pos_yellow)
{
    this->ball->setBodyPosition(ball[0], ball[1], 0);
    dBodySetLinearVel(this->ball->body, ball[2], ball[3], 0);
    dBodySetAngularVel(this->ball->body, 0, 0, 0);
    std::vector<std::vector<double>> blues;
    blues.clear();
    for (int i = 0; i < this->field.getRobotsBlueCount() * 3; i = i + 3)
    {
        std::vector<double> pos;
        pos.clear();
        pos.push_back(pos_blue[i]);
        pos.push_back(pos_blue[i + 1]);
        pos.push_back(pos_blue[i + 2]);
        blues.push_back(pos);
    }

    std::vector<std::vector<double>> yellows;
    yellows.clear();
    for (int i = 0; i < this->field.getRobotsYellowCount() * 3; i = i + 3)
    {
        std::vector<double> pos;
        pos.clear();
        pos.push_back(pos_yellow[i]);
        pos.push_back(pos_yellow[i + 1]);
        pos.push_back(pos_yellow[i + 2]);
        yellows.push_back(pos);
    }

    for (uint32_t i = 0; i < this->field.getRobotsBlueCount(); i++)
    {
        this->robots[i]->resetRobot();
        this->robots[i]->setXY(blues[i][0], blues[i][1]);
        this->robots[i]->setDir(blues[i][2]);
    }
    for (uint32_t i = this->field.getRobotsBlueCount(); i < this->field.getRobotsYellowCount() + this->field.getRobotsBlueCount(); i++)
    {
        uint32_t k = i - this->field.getRobotsBlueCount();
        this->robots[i]->resetRobot();
        this->robots[i]->setXY(yellows[k][0], yellows[k][1]);
        this->robots[i]->setDir(yellows[k][2]);
    }
}
