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
        for (int i = 0; i < world->field.getRobotsCount(); i++)
        {
            double *a = new double[8];
            for(int j = 0; j < 8; j++) a[j] = act[i*8 + j];
            actions.push_back(a);
        }
        world->step(actions);
        for (auto a : actions) delete a;
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
}
