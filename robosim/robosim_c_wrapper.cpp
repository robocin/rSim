#include "../src/world.h"

extern "C"
{
  World *World_new() { return new World(); }
  void World_del(World *world) { delete world; }
  void step(World *world, double *act, double *state_data)
  {
    std::vector<std::tuple<double, double>> actions;
    actions.clear();
    for (int i = 0; i < 12; i = i + 2)
    {
      std::tuple<double, double> action(act[i], act[i + 1]);
      actions.push_back(action);
    }
    world->step(Config::World().getDeltaTime(), actions);
    const std::vector<double> state = world->getState();
    const double *state_features = state.data();
    memcpy(state_data, state_features, state.size() * sizeof(double));
  }
}