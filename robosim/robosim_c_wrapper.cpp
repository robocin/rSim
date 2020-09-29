#include "../src/world.h"

extern "C"
{
    World *newWorld(int fieldType, int nRobotsBlue, int nRobotsYellow)
    {
        return new World(fieldType, nRobotsBlue, nRobotsYellow);
    }
    void delWorld(World *world) { delete world; }
    void step(World *world, double *act)
    {
        std::vector<std::tuple<double, double>> actions;
        actions.clear();
        for (int i = 0; i < 12; i = i + 2)
        {
            std::tuple<double, double> action(act[i], act[i + 1]);
            actions.push_back(action);
        }
        world->step(Config::World().getDeltaTime(), actions);
    }
    void getState(World *world, double *state_data)
    {
        const std::vector<double> state = world->getState();
        const double *state_features = state.data();
        memcpy(state_data, state_features, state.size() * sizeof(double));
    }
    void getFieldParams(World *world, double *params_data)
    {
        const std::vector<double> params = world->getFieldParams();
        const double *params_features = params.data();
        memcpy(params_data, params_features, params.size() * sizeof(double));
    }
    int getEpisodeTime(World *world) { return world->getEpisodeTime(); }
    int getGoalsBlue(World *world) { return world->getGoals()[0]; }
    int getGoalsYellow(World *world) { return world->getGoals()[1]; }
    void replace(World *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace(ball_data, pos_blue_data, pos_yellow_data);
    }
}
