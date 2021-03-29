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
        for (int i = 0; i < world->field.getRobotsCount(); i++)
        {
            std::tuple<double, double> action(act[i*2], act[i*2 + 1]);
            actions.push_back(action);
        }
        world->step(actions);
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
}