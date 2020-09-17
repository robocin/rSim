#include "speed_estimator.h"

speedEstimator::speedEstimator(bool t_have_angle, double t_beta, double t_acc_th)
{
    this->prev_time = 0;
    this->prev_pos[3] = {0.0};
    this->have_angle = t_have_angle;
    this->count_avg = 0;
    this->prev_lin = 0.0;
    this->acc_th = t_acc_th;
    this->prev_vel_x = 0.0;
    this->prev_vel_y = 0.0;
    this->prev_vel_yaw = 0.0;
    this->moving_avg_x = 0.0;
    this->moving_avg_y = 0.0;
    this->moving_avg_yaw = 0.0;
    this->corr_moving_avg_yaw = 0.0;
    this->corr_moving_avg_x = 0.0;
    this->corr_moving_avg_y = 0.0;
    this->beta = t_beta;
}
double to_positive_angle(double angle)
{
    return fmod(angle + 2 * M_PI, 2 * M_PI);
}

double smallest_angle_diff(double target, double source)
{
    double angle = to_positive_angle(target) - to_positive_angle(source);

    if (angle > M_PI)
    {
        angle = angle - 2 * M_PI;
    }

    if (angle < -M_PI)
    {
        angle = angle + 2 * M_PI;
    }

    return angle;
}

void speedEstimator::estimateSpeed(double time, dReal *pose, dReal *vel)
{
    time = time / 1000;
    int scale = 1;
    if (this->prev_time == 0 || (time - this->prev_time) == 0)
    {
        this->prev_time = time;
        this->prev_pos[0] = pose[0];
        this->prev_pos[1] = pose[1];
        if (this->have_angle)
        {
            this->prev_pos[2] = pose[2];
            vel[0] = 0.0;
            vel[1] = 0.0;
            vel[2] = 0.0;
        }
        else
        {
            vel[0] = 0.0;
            vel[1] = 0.0;
        }
    }

    else
    {

        double dt = (time - this->prev_time);
        this->count_avg += 1;
        double vel_x = (pose[0] - this->prev_pos[0]) / dt;
        double vel_y = (pose[1] - this->prev_pos[1]) / dt;
        double vel_yaw;
        if (this->have_angle)
        {
            vel_yaw = -smallest_angle_diff(pose[2], this->prev_pos[2]) / dt;
        }
        double lin = sqrt(pow(vel_x, 2) + pow(vel_y, 2));
        if (this->prev_lin != 0.0)
        {
            if (abs(lin - this->prev_lin) / dt > this->acc_th)
            {
                lin = 0.0;
                vel_x = this->prev_vel_x;
                vel_y = this->prev_vel_y;
                vel_yaw = this->prev_vel_yaw;
            }
        }
        this->moving_avg_x = this->beta * this->moving_avg_x + (1 - this->beta) * vel_x;
        this->corr_moving_avg_x = this->moving_avg_x / (1 - pow(this->beta, this->count_avg));
        this->moving_avg_y = this->beta * this->moving_avg_y + (1 - this->beta) * vel_y;
        this->corr_moving_avg_y = this->moving_avg_y / (1 - pow(this->beta, this->count_avg));
        if (this->have_angle)
        {
            this->moving_avg_yaw = this->beta * this->moving_avg_yaw + (1 - this->beta) * vel_yaw;
            this->corr_moving_avg_yaw = this->moving_avg_yaw / (1 - pow(this->beta, this->count_avg));
            this->prev_pos[0] = pose[0];
            this->prev_pos[1] = pose[1];
            this->prev_pos[2] = pose[2];
        }

        else
        {
            this->prev_pos[0] = pose[0];
            this->prev_pos[1] = pose[1];
            this->prev_pos[2] = 0;
        }
        this->prev_time = time;
        this->prev_lin = lin;
        this->prev_vel_x = vel_x;
        this->prev_vel_y = vel_y;

        if (this->have_angle)
        {
            vel[0] = scale * this->corr_moving_avg_x;
            vel[1] = scale * this->corr_moving_avg_y;
            vel[2] = scale * this->corr_moving_avg_yaw;
        }

        else
        {
            vel[0] = scale * this->corr_moving_avg_x;
            vel[1] = scale * this->corr_moving_avg_y;
            vel[2] = 0.0;
        }
    }
}