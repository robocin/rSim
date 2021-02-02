#include "../src/sslworld.h"

extern "C"
{
    SSLWorld *newWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, int timeStep_ms,
                    double *ballPos, double *blueRobotsPos, double *yellowRobotsPos)
    {
        return new SSLWorld(fieldType, nRobotsBlue, nRobotsYellow, timeStep_ms / 1000.0,
                         ballPos, blueRobotsPos, yellowRobotsPos);
    }
    void delWorld(SSLWorld *world) { delete world; }
    void step(SSLWorld *world, double *act)
    {
        std::vector<double*> actions;
        actions.clear();
        
        for (int i = 0; i < SSLConfig::Field().getRobotsCount(); i = i + 6)
        {
            double a[] = {act[i], act[i + 1], act[i + 2], act[i + 3], act[i + 4], act[i + 5]};
            actions.push_back(a);
        }
        world->step(world->getTimeStep(), actions);
    }
    void getState(SSLWorld *world, double *state_data)
    {
        const std::vector<double> state = world->getState();
        const double *state_features = state.data();
        memcpy(state_data, state_features, state.size() * sizeof(double));
    }
    void getFieldParams(SSLWorld *world, double *params_data)
    {
        const std::vector<double> params = world->getFieldParams();
        const double *params_features = params.data();
        memcpy(params_data, params_features, params.size() * sizeof(double));
    }
    void replace(SSLWorld *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace(ball_data, pos_blue_data, pos_yellow_data);
    }
    void replace_with_vel(SSLWorld *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace_with_vel(ball_data, pos_blue_data, pos_yellow_data);
    }
}
