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
dReal randn_notrig(dReal mu = 0.0, dReal sigma = 1.0);
dReal randn_trig(dReal mu = 0.0, dReal sigma = 1.0);
dReal rand0_1();

dReal fric(dReal f)
{
    if (f + 1 < 0.001)
        return dInfinity;
    return f;
}

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

World::World()
{
    this->episodeSteps = 0;
    this->faultSteps = 0;
    _world = this;
    this->updatedCursor = false;
    physics = new PWorld(0.05, 9.81f, Config::Field().getRobotsCount());
    ball = new PBall(0, 0, 0.5, Config::World().getBallRadius(), Config::World().getBallMass());

    ground = new PGround(Config::Field().getFieldRad(), Config::Field().getFieldLength(), Config::Field().getFieldWidth(),
                         Config::Field().getFieldPenaltyDepth(), Config::Field().getFieldPenaltyWidth(), Config::Field().getFieldPenaltyPoint(),
                         Config::Field().getFieldLineWidth(), 0);


    const double thick = Config::Field().getWallThickness();
    const double increment = thick / 2; //cfg->Field_Margin() + cfg->Field_Referee_Margin() + thick / 2;
    const double pos_x = Config::Field().getFieldLength() / 2.0 + increment;
    const double pos_y = Config::Field().getFieldWidth() / 2.0 + increment;
    const double pos_z = 0.0;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;
    const double tone = 1.0;

    const double gthick = Config::Field().getWallThickness();
    const double gpos_x = (Config::Field().getFieldLength() + gthick) / 2.0 + Config::Field().getGoalDepth();
    const double gpos_y = (Config::Field().getGoalWidth() + gthick) / 2.0;
    const double gpos_z = 0; //Config::Field().getGoalHeight() / 2.0;
    const double gsiz_x = Config::Field().getGoalDepth() + gthick;
    const double gsiz_y = Config::Field().getGoalWidth();
    const double gsiz_z = siz_z; //Config::Field().getGoalHeight();
    const double gpos2_x = (Config::Field().getFieldLength() + gsiz_x) / 2.0;

    // Bounding walls

    for (auto &w : walls)
    {
        w = new PFixedBox(thick / 2, pos_y, pos_z,
                          siz_x, thick, siz_z);
    }

    walls[0] = new PFixedBox(thick / 2, pos_y, pos_z,
                             siz_x, thick, siz_z);

    walls[1] = new PFixedBox(-thick / 2, -pos_y, pos_z,
                             siz_x, thick, siz_z);

    walls[2] = new PFixedBox(pos_x, gpos_y + (siz_y - gsiz_y) / 4, pos_z,
                             thick, (siz_y - gsiz_y) / 2, siz_z);

    walls[10] = new PFixedBox(pos_x, -gpos_y - (siz_y - gsiz_y) / 4, pos_z,
                              thick, (siz_y - gsiz_y) / 2, siz_z);

    walls[3] = new PFixedBox(-pos_x, gpos_y + (siz_y - gsiz_y) / 4, pos_z,
                             thick, (siz_y - gsiz_y) / 2, siz_z);

    walls[11] = new PFixedBox(-pos_x, -gpos_y - (siz_y - gsiz_y) / 4, pos_z,
                              thick, (siz_y - gsiz_y) / 2, siz_z);

    // Goal walls
    walls[4] = new PFixedBox(gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z);

    walls[5] = new PFixedBox(gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);

    walls[6] = new PFixedBox(gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);

    walls[7] = new PFixedBox(-gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z);

    walls[8] = new PFixedBox(-gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);

    walls[9] = new PFixedBox(-gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);

    // Corner Wall
    walls[12] = new PFixedBox(-pos_x + gsiz_x / 2.8, pos_y - gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z);
    walls[12]->setRotation(0, 0, 1, M_PI / 4);

    walls[13] = new PFixedBox(pos_x - gsiz_x / 2.8, pos_y - gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z);
    walls[13]->setRotation(0, 0, 1, -M_PI / 4);

    walls[14] = new PFixedBox(pos_x - gsiz_x / 2.8, -pos_y + gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z);
    walls[14]->setRotation(0, 0, 1, M_PI / 4);

    walls[15] = new PFixedBox(-pos_x + gsiz_x / 2.8, -pos_y + gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z);
    walls[15]->setRotation(0, 0, 1, -M_PI / 4);

    physics->addObject(ground);
    physics->addObject(ball);
    for (auto &wall : walls)
        physics->addObject(wall);
    const int wheeltexid = 4 * Config::Field().getRobotsCount() + 12 + 1; //37 for 6 robots
    srand(static_cast<unsigned>(time(0)));

    // gonca comentou pra ver aqui
    // cfg->robotSettings = cfg->blueSettings;
    for (int k = 0; k < Config::Field().getRobotsCount() * 2; k++)
    {
        bool turn_on = (k % Config::Field().getRobotsCount() < 3) ? true : false;
        float LO_X = -0.65;
        float LO_Y = -0.55;
        float HI_X = 0.65;
        float HI_Y = 0.55;
        float dir = 1.0;
        // if (k > Config::Field().getRobotsCount())
        // {
        //     cfg->robotSettings = cfg->yellowSettings;
        //     x = form->x[k - Config::Field().getRobotsCount()];
        //     y = -form->y[k - Config::Field().getRobotsCount()];
        //     dir = -1;
        // }
        float x = LO_X + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_X - LO_X)));
        float y = LO_Y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_Y - LO_Y)));
        x = (k % Config::Field().getRobotsCount() < 3) ? x : 3.0;
        y = (k % Config::Field().getRobotsCount() < 3) ? y : 3.0;
        robots[k] = new CRobot(
            physics, ball, x, y, ROBOT_START_Z(),
            k + 1, wheeltexid, dir, turn_on);
    }

    physics->initAllObjects();

    //Surfaces

    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1; // | dContactSlip1;
    ballwithwall.surface.mu = 1;                                  //fric(cfg->ballfriction());
    ballwithwall.surface.bounce = Config::World().getBallBounce();
    ballwithwall.surface.bounce_vel = Config::World().getBallBounceVel();
    ballwithwall.surface.slip1 = 0; //cfg->ballslip();

    PSurface wheelswithground;
    PSurface *ball_ground = physics->createSurface(ball, ground);
    ball_ground->surface = ballwithwall.surface;
    ball_ground->callback = ballCallBack;

    for (auto &wall : walls)
        physics->createSurface(ball, wall)->surface = ballwithwall.surface;

    for (int k = 0; k < 2 * Config::Field().getRobotsCount(); k++)
    {
        physics->createSurface(robots[k]->chassis, ground);
        for (auto &wall : walls)
            physics->createSurface(robots[k]->chassis, wall);
        physics->createSurface(robots[k]->dummy, ball);
        //physics->createSurface(robots[k]->chassis,ball);
        for (auto &wheel : robots[k]->wheels)
        {
            physics->createSurface(wheel->cyl, ball);
            PSurface *w_g = physics->createSurface(wheel->cyl, ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (auto &b : robots[k]->balls)
        {
            //            physics->createSurface(b->pBall,ball);
            PSurface *w_g = physics->createSurface(b->pBall, ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (int j = k + 1; j < 2 * Config::Field().getRobotsCount(); j++)
        {
            if (k != j)
            {
                physics->createSurface(robots[k]->dummy, robots[j]->dummy); //seams ode doesn't understand cylinder-cylinder contacts, so I used spheres
            }
        }
    }

    in_buffer = new char[65536];
    ball_speed_estimator = new speedEstimator(false, 0.95, 100000);
    for (int i = 0; i < Config::Field().getRobotsCount(); i++)
    {
        blue_speed_estimator[i] = new speedEstimator(true, 0.95, 100000);
        yellow_speed_estimator[i] = new speedEstimator(true, 0.95, 100000);
    }

    // initialize robot state
    for (int team = 0; team < TEAM_COUNT; ++team)
    {
        for (int i = 0; i < MAX_ROBOT_COUNT; ++i)
        {
            lastInfraredState[team][i] = false;
            lastKickState[team][i] = NO_KICK;
        }
    }
}

int World::robotIndex(unsigned int robot, int team)
{
    if (robot >= Config::Field().getRobotsCount())
        return -1;
    return robot + team * Config::Field().getRobotsCount();
}

World::~World()
{
    delete physics;
}

void World::step(dReal dt, std::vector<std::tuple<double, double>> actions)
{
    setActions(actions);

    // Pq ele faz isso 5 vezes?
    // - Talvez mais precisao (Ele sempre faz um step de dt*0.2 )
    for (int kk = 0; kk < 5; kk++)
    {
        const dReal *ballvel = dBodyGetLinearVel(ball->body);
        // Norma do vetor velocidade da bola
        dReal ballspeed = ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2];
        ballspeed = sqrt(ballspeed);
        dReal ballfx = 0, ballfy = 0, ballfz = 0;
        dReal balltx = 0, ballty = 0, balltz = 0;
        if (ballspeed < 0.01)
        {
            ; //const dReal* ballAngVel = dBodyGetAngularVel(ball->body);
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
            dBodyAddTorque(ball->body, balltx, ballty, balltz);
        }
        dBodyAddForce(ball->body, ballfx, ballfy, ballfz);

        physics->step(dt * 0.2, fullSpeed);
    }

    for (int k = 0; k < Config::Field().getRobotsCount() * 2; k++)
    {
        robots[k]->step();
    }
    this->episodeSteps++;
    posProcess();
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

dReal normalizeAngle(dReal a)
{
    if (a > 180)
        return -360 + a;
    if (a < -180)
        return 360 + a;
    return a;
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
    std::vector<double> field = std::vector<double>(static_cast<std::size_t>(4));
    field.clear();
    field.push_back(Config::Field().getFieldWidth());
    field.push_back(Config::Field().getFieldLength());
    field.push_back(Config::Field().getGoalDepth());
    field.push_back(Config::Field().getGoalWidth());
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
    for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
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

void World::posProcess()
{
    bool side;
    bool is_goal = false;
    bool out_of_bands = false;

    dReal bx, by, bz;
    ball->getBodyPosition(bx, by, bz);
    // Goal Detection
    if (bx > 0.75 && abs(by) < 0.2)
    {
        side = true;
        goalsBlue++;
        is_goal = true;
    }
    else if (bx < -0.75 && abs(by) < 0.2)
    {
        side = false;
        goalsYellow++;
        is_goal = true;
    }
    if (bx < -2 || bx > 2 || by > 2 || by < -2)
        out_of_bands = true;

    bool penalty = false;
    bool goal_shot = false;
    if (bx < -0.6 && abs(by) < 0.35)
    {
        // Penalti Detection
        bool one_in_pen_area = false;
        for (uint32_t i = 0; i < Config::Field().getRobotsCount(); i++)
        {
            int num = robotIndex(i, 0);
            if (!robots[num]->on)
                continue;
            dReal rx, ry;
            robots[num]->getXY(rx, ry);
            if (rx < -0.6 && abs(ry) < 0.35)
            {
                if (one_in_pen_area)
                {
                    penalty = true;
                    side = true;
                }
                else
                    one_in_pen_area = true;
            }
        }

        // Atk Fault Detection
        if (withGoalKick)
        {
            bool one_in_enemy_area = false;
            for (uint32_t i = 0; i < Config::Field().getRobotsCount(); i++)
            {
                int num = robotIndex(i, 1);
                if (!robots[num]->on)
                    continue;
                dReal rx, ry;
                robots[num]->getXY(rx, ry);
                if (rx < -0.6 && abs(ry) < 0.35)
                {
                    if (one_in_enemy_area)
                    {
                        goal_shot = true;
                        side = false;
                    }
                    else
                        one_in_enemy_area = true;
                }
            }
        }
    }

    if (bx > 0.6 && abs(by) < 0.35)
    {
        // Penalti Detection
        bool one_in_pen_area = false;
        for (uint32_t i = 0; i < Config::Field().getRobotsCount(); i++)
        {
            int num = robotIndex(i, 1);
            if (!robots[num]->on)
                continue;
            dReal rx, ry;
            robots[num]->getXY(rx, ry);
            if (rx > 0.6 && abs(ry) < 0.35)
            {
                if (one_in_pen_area)
                {
                    penalty = true;
                    side = false;
                }
                else
                    one_in_pen_area = true;
            }
        }

        // Atk Fault Detection
        if (withGoalKick)
        {
            bool one_in_enemy_area = false;
            for (uint32_t i = 0; i < Config::Field().getRobotsCount(); i++)
            {
                int num = robotIndex(i, 0);
                if (!robots[num]->on)
                    continue;
                dReal rx, ry;
                robots[num]->getXY(rx, ry);
                if (rx > 0.6 && abs(ry) < 0.35)
                {
                    if (one_in_enemy_area)
                    {
                        goal_shot = true;
                        side = true;
                    }
                    else
                        one_in_enemy_area = true;
                }
            }
        }
    }

    // Fault Detection
    bool fault = false;
    this->faultSteps++;
    int quadrant = 4;
    if (this->faultSteps * Config::World().getDeltaTime() * 1000 >= 10000)
    {
        if (fabs(ball_prev_pos.first - bx) < 0.0001 &&
            fabs(ball_prev_pos.second - by) < 0.0001)
        {
            if ((bx < -0.6) && abs(by) < 0.35)
            {
                penalty = true;
                side = true;
            }
            else if (bx > 0.6 && abs(by) < 0.35)
            {

                penalty = true;
                side = false;
            }
            else
            {
                fault = true;
                if (bx < 0 && by <= 0)
                {
                    quadrant = 0;
                }
                else if (bx < 0 && by >= 0)
                {
                    quadrant = 1;
                }
                else if (bx > 0 && by <= 0)
                {
                    quadrant = 2;
                }
                else if (bx > 0 && by >= 0)
                {
                    quadrant = 3;
                }
            }
        }
        ball_prev_pos.first = bx;
        ball_prev_pos.second = by;
        this->faultSteps = 0;
    }
    else
    {
        if (fabs(ball_prev_pos.first - bx) > 0.0001 ||
            fabs(ball_prev_pos.second - by) > 0.0001)
        {
            this->faultSteps = 0;
        }
    }
    ball_prev_pos.first = bx;
    ball_prev_pos.second = by;

    // End Time Detection
    bool end_time;

    if ((getEpisodeTime()) > 300000)
    {
        end_time = true;
    }
    else
    {
        end_time = false;
    }

    if ((((getEpisodeTime()) / 60000) - minute) > 0)
    {
        minute++;
        std::cout << "****************** " << minute << " Minutes ****************" << std::endl;
    }

    if (randomStart && (is_goal || penalty || fault || goal_shot || end_time))
    {
        dReal x, y;
        for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
        {
            if (!robots[i]->on)
                continue;
            getValidPosition(x, y, i);
            robots[i]->setXY(x, y);
        }
        getValidPosition(x, y, Config::Field().getRobotsCount() * 2);
        ball->setBodyPosition(x, y, 0);
        dBodySetLinearVel(ball->body, 0, 0, 0);
        dBodySetAngularVel(ball->body, 0, 0, 0);

        this->faultSteps = 0;
        if (end_time)
        {
            this->episodeSteps = 0;
            goalsBlue = 0;
            goalsYellow = 0;
            minute = 0;
        }
    }
    else if (is_goal || end_time)
    {
        ball->setBodyPosition(0, 0, 0);
        dBodySetLinearVel(ball->body, 0, 0, 0);
        dBodySetAngularVel(ball->body, 0, 0, 0);

        if (side)
        {
            dReal posX[6] = {0.15, 0.35, 0.71, -0.08, -0.35, -0.71};
            dReal posY[6] = {0.02, 0.13, -0.02, 0.02, 0.13, -0.02};

            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i] * (-1), posY[i]);
            }
        }
        else
        {
            dReal posX[6] = {0.08, 0.35, 0.71, -0.15, -0.35, -0.71};
            dReal posY[6] = {0.02, 0.13, -0.02, 0.02, 0.13, -0.02};
            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i] * (-1), posY[i]);
            }
        }
        if (end_time)
        {
            this->faultSteps = 0;
            this->episodeSteps = 0;
            goalsBlue = 0;
            goalsYellow = 0;
            minute = 0;
        }
    }
    else if (fault)
    {
        if (quadrant == 0)
        {
            ball->setBodyPosition(-0.375, -0.4, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);

            dReal posX[6] = {-0.575, -0.44, -0.71, -0.175, -0.3, 0.71};
            dReal posY[6] = {-0.4, 0.13, -0.02, -0.4, 0.13, -0.02};

            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i], posY[i]);
            }
        }
        else if (quadrant == 1)
        {
            ball->setBodyPosition(-0.375, 0.4, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);

            dReal posX[6] = {-0.575, -0.44, -0.71, -0.175, -0.30, 0.71};
            dReal posY[6] = {0.4, -0.13, -0.02, 0.4, -0.13, -0.02};

            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i], posY[i]);
            }
        }
        else if (quadrant == 2)
        {
            ball->setBodyPosition(0.375, -0.4, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);

            dReal posX[6] = {0.175, 0.3, -0.71, 0.575, 0.44, 0.71};
            dReal posY[6] = {-0.4, 0.13, -0.02, -0.4, 0.13, -0.02};

            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i], posY[i]);
            }
        }
        else if (quadrant == 3)
        {
            ball->setBodyPosition(0.375, 0.4, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);

            dReal posX[6] = {0.175, 0.3, -0.71, 0.575, 0.44, 0.71};
            dReal posY[6] = {0.4, -0.13, -0.02, 0.4, -0.13, -0.02};

            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i], posY[i]);
            }
        }
        this->faultSteps = 0;
    }
    else if (penalty)
    {

        this->faultSteps = 0;

        if (side)
        {
            dReal posX[6] = {0.75, -0.06, -0.06, 0.35, -0.05, -0.74};
            dReal posY[6] = {-0.01, 0.23, -0.33, 0.02, 0.48, 0.01};

            ball->setBodyPosition(-0.47, -0.01, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);

            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i] * (-1), posY[i]);
            }
        }
        else
        {
            dReal posX[6] = {0.35, -0.05, -0.74, 0.75, -0.06, -0.06};
            dReal posY[6] = {0.02, 0.48, 0.01, -0.01, 0.23, -0.33};

            ball->setBodyPosition(0.47, -0.01, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);
            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i], posY[i]);
            }
        }
    }
    else if (goal_shot)
    {

        this->faultSteps = 0;

        dReal posX[6] = {0.65, 0.48, 0.49, 0.19, 0.18, -0.67};
        dReal posY[6] = {0.11, 0.37, -0.33, 0.13, -0.21, -0.01};
        if (side)
        {
            ball->setBodyPosition(0.61, 0.11, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);
            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i], posY[i]);
            }
        }
        else
        {
            ball->setBodyPosition(-0.61, 0.11, 0);
            dBodySetLinearVel(ball->body, 0, 0, 0);
            dBodySetAngularVel(ball->body, 0, 0, 0);
            for (uint32_t i = 0; i < Config::Field().getRobotsCount() * 2; i++)
            {
                robots[i]->setXY(posX[i] * (-1), posY[i]);
            }
        }
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
    max = max > 0 ? max : Config::Field().getRobotsCount() * 2;
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

void RobotsFormation::setAll(const dReal *xx, const dReal *yy)
{
    for (int i = 0; i < Config::Field().getRobotsCount(); i++)
    {
        x[i] = xx[i];
        y[i] = yy[i];
    }
}

RobotsFormation::RobotsFormation(int type)
{
    if (type == 0)
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {2.2, 1.0, 1.0, 1.0, 0.33, 1.22,
                                           3, 3.2, 3.4, 3.6, 3.8, 4.0};
        dReal teamPosY[MAX_ROBOT_COUNT] = {0.0, -0.75, 0.0, 0.75, 0.25, 0.0,
                                           1, 1, 1, 1, 1, 1};
        setAll(teamPosX, teamPosY);
    }
    if (type == 1) // formation 1
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {1.5, 1.5, 1.5, 0.55, 2.5, 3.6,
                                           3.2, 3.2, 3.2, 3.2, 3.2, 3.2};
        dReal teamPosY[MAX_ROBOT_COUNT] = {1.12, 0.0, -1.12, 0.0, 0.0, 0.0,
                                           0.75, -0.75, 1.5, -1.5, 2.25, -2.25};
        setAll(teamPosX, teamPosY);
    }
    if (type == 2) // formation 2
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {4.2, 3.40, 3.40, 0.7, 0.7, 0.7,
                                           2, 2, 2, 2, 2, 2};
        dReal teamPosY[MAX_ROBOT_COUNT] = {0.0, -0.20, 0.20, 0.0, 2.25, -2.25,
                                           0.75, -0.75, 1.5, -1.5, 2.25, -2.25};
        setAll(teamPosX, teamPosY);
    }
    if (type == 3) // outside field
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {0.4, 0.8, 1.2, 1.6, 2.0, 2.4,
                                           2.8, 3.2, 3.6, 4.0, 4.4, 4.8};
        dReal teamPosY[MAX_ROBOT_COUNT] = {-4.0, -4.0, -4.0, -4.0, -4.0, -4.0,
                                           -4.0, -4.0, -4.0, -4.0, -4.0, -4.0};
        setAll(teamPosX, teamPosY);
    }
    if (type == 4)
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {2.8, 2.5, 2.5, 0.8, 0.8, 1.1, 3, 3.2, 3.4, 3.6, 3.8, 4.0};
        dReal teamPosY[MAX_ROBOT_COUNT] = {5 + 0.0, 5 - 0.3, 5 + 0.3, 5 + 0.0, 5 + 1.5, 5.5, 1, 1, 1, 1, 1, 1};
        setAll(teamPosX, teamPosY);
    }
    if (type == -1) // outside
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {0.4, 0.8, 1.2, 1.6, 2.0, 2.4,
                                           2.8, 3.2, 3.6, 4.0, 4.4, 4.8};
        dReal teamPosY[MAX_ROBOT_COUNT] = {-3.4, -3.4, -3.4, -3.4, -3.4, -3.4,
                                           -3.4, -3.4, -3.4, -3.4, -3.4, -3.4};
        setAll(teamPosX, teamPosY);
    }
}

void RobotsFormation::loadFromFile(const QString &filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file);
    int k;
    for (k = 0; k < Config::Field().getRobotsCount(); k++)
        x[k] = y[k] = 0;
    k = 0;
    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList list = line.split(",");
        if (list.count() >= 2)
        {
            x[k] = list[0].toFloat();
            y[k] = list[1].toFloat();
        }
        else if (list.count() == 1)
        {
            x[k] = list[0].toFloat();
        }
        if (k == Config::Field().getRobotsCount() - 1)
            break;
        k++;
    }
}

void RobotsFormation::resetRobots(CRobot **r, int team)
{
    dReal dir = -1;
    if (team == 1)
        dir = 1;
    for (int k = 0; k < Config::Field().getRobotsCount(); k++)
    {
        r[k + team * Config::Field().getRobotsCount()]->setXY(x[k] * dir, y[k]);
        r[k + team * Config::Field().getRobotsCount()]->resetRobot();
    }
}

//// Copy & pasted from http://www.dreamincode.net/code/snippet1446.htm
/******************************************************************************/
/* randn()
 *
 * Normally (Gaussian) distributed random numbers, using the Box-Muller
 * transformation.  This transformation takes two uniformly distributed deviates
 * within the unit circle, and transforms them into two independently
 * distributed normal deviates.  Utilizes the internal rand() function; this can
 * easily be changed to use a better and faster RNG.
 *
 * The parameters passed to the function are the mean and standard deviation of
 * the desired distribution.  The default values used, when no arguments are
 * passed, are 0 and 1 - the standard normal distribution.
 *
 *
 * Two functions are provided:
 *
 * The first uses the so-called polar version of the B-M transformation, using
 * multiple calls to a uniform RNG to ensure the initial deviates are within the
 * unit circle.  This avoids making any costly trigonometric function calls.
 *
 * The second makes only a single set of calls to the RNG, and calculates a
 * position within the unit circle with two trigonometric function calls.
 *
 * The polar version is generally superior in terms of speed; however, on some
 * systems, the optimization of the math libraries may result in better
 * performance of the second.  Try it out on the target system to see which
 * works best for you.  On my test machine (Athlon 3800+), the non-trig version
 * runs at about 3x10^6 calls/s; while the trig version runs at about
 * 1.8x10^6 calls/s (-O2 optimization).
 *
 *
 * Example calls:
 * randn_notrig();	//returns normal deviate with mean=0.0, std. deviation=1.0
 * randn_notrig(5.2,3.0);	//returns deviate with mean=5.2, std. deviation=3.0
 *
 *
 * Dependencies - requires <cmath> for the sqrt(), sin(), and cos() calls, and a
 * #defined value for PI.
 */

/******************************************************************************/
//	"Polar" version without trigonometric calls
dReal randn_notrig(dReal mu, dReal sigma)
{
    if (sigma == 0)
        return mu;
    static bool deviateAvailable = false; //	flag
    static dReal storedDeviate;           //	deviate from previous calculation
    dReal polar, rsquared, var1, var2;

    //	If no deviate has been stored, the polar Box-Muller transformation is
    //	performed, producing two independent normally-distributed random
    //	deviates.  One is stored for the next round, and one is returned.
    if (!deviateAvailable)
    {

        //	choose pairs of uniformly distributed deviates, discarding those
        //	that don't fall within the unit circle
        do
        {
            var1 = 2.0 * (dReal(rand()) / dReal(RAND_MAX)) - 1.0;
            var2 = 2.0 * (dReal(rand()) / dReal(RAND_MAX)) - 1.0;
            rsquared = var1 * var1 + var2 * var2;
        } while (rsquared >= 1.0 || rsquared == 0.0);

        //	calculate polar tranformation for each deviate
        polar = sqrt(-2.0 * log(rsquared) / rsquared);

        //	store first deviate and set flag
        storedDeviate = var1 * polar;
        deviateAvailable = true;

        //	return second deviate
        return var2 * polar * sigma + mu;
    }

    //	If a deviate is available from a previous call to this function, it is
    //	returned, and the flag is set to false.
    else
    {
        deviateAvailable = false;
        return storedDeviate * sigma + mu;
    }
}

/******************************************************************************/
//	Standard version with trigonometric calls
#define PI 3.14159265358979323846

dReal randn_trig(dReal mu, dReal sigma)
{
    static bool deviateAvailable = false; //	flag
    static dReal storedDeviate;           //	deviate from previous calculation
    dReal dist, angle;

    //	If no deviate has been stored, the standard Box-Muller transformation is
    //	performed, producing two independent normally-distributed random
    //	deviates.  One is stored for the next round, and one is returned.
    if (!deviateAvailable)
    {

        //	choose a pair of uniformly distributed deviates, one for the
        //	distance and one for the angle, and perform transformations
        dist = sqrt(-2.0 * log(dReal(rand()) / dReal(RAND_MAX)));
        angle = 2.0 * PI * (dReal(rand()) / dReal(RAND_MAX));

        //	calculate and store first deviate and set flag
        storedDeviate = dist * cos(angle);
        deviateAvailable = true;

        //	calculate return second deviate
        return dist * sin(angle) * sigma + mu;
    }

    //	If a deviate is available from a previous call to this function, it is
    //	returned, and the flag is set to false.
    else
    {
        deviateAvailable = false;
        return storedDeviate * sigma + mu;
    }
}

dReal rand0_1()
{
    return (dReal)(rand()) / (dReal)(RAND_MAX);
}
