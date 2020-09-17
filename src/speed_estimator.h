#ifndef SPEED_H
#define SPEED_H

#include "physics/pworld.h"
#include "physics/pcylinder.h"
#include "physics/pbox.h"
#include "physics/pball.h"

class speedEstimator
{

public:
    speedEstimator(bool t_have_angle, double t_beta, double t_acc_th);
    void estimateSpeed(double time, dReal *pose, dReal *vel);

private:
    double prev_time;
    double prev_pos[3];
    bool have_angle;
    int count_avg;
    double prev_lin;
    double acc_th;
    double prev_vel_x;
    double prev_vel_y;
    double prev_vel_yaw;
    double moving_avg_x;
    double moving_avg_y;
    double moving_avg_yaw;
    double corr_moving_avg_yaw;
    double corr_moving_avg_x;
    double corr_moving_avg_y;
    double beta;
};

#endif // SPEED_H