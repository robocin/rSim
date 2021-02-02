#include "../src/vssworld.h"

extern "C"
{
    VSSWorld *newWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, int timeStep_ms,
                    double *ballPos, double *blueRobotsPos, double *yellowRobotsPos)
    {
        return new VSSWorld(fieldType, nRobotsBlue, nRobotsYellow, timeStep_ms / 1000.0,
                         ballPos, blueRobotsPos, yellowRobotsPos);
    }
    void delWorld(VSSWorld *world) { delete world; }
    void step(VSSWorld *world, double *act)
    {
        std::vector<std::tuple<double, double>> actions;
        actions.clear();
        for (int i = 0; i < 12; i = i + 2)
        {
            std::tuple<double, double> action(act[i], act[i + 1]);
            actions.push_back(action);
        }
        world->step(world->getTimeStep(), actions);
    }
    void getState(VSSWorld *world, double *state_data)
    {
        const std::vector<double> state = world->getState();
        const double *state_features = state.data();
        memcpy(state_data, state_features, state.size() * sizeof(double));
    }
    void getFieldParams(VSSWorld *world, double *params_data)
    {
        const std::vector<double> params = world->getFieldParams();
        const double *params_features = params.data();
        memcpy(params_data, params_features, params.size() * sizeof(double));
    }
    void replace(VSSWorld *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace(ball_data, pos_blue_data, pos_yellow_data);
    }
    void replace_with_vel(VSSWorld *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace_with_vel(ball_data, pos_blue_data, pos_yellow_data);
    }
}