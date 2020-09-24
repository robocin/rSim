#include <iostream>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>
#include <boost/python.hpp>
#include <string>
#include <vector>
#include "world.h"

using namespace boost::python;
class RoboSim
{
private:
  World *world = new World();

public:
  RoboSim();
  void del() { delete world; }
  boost::python::list step(boost::python::list actions)
  {
    std::vector<std::tuple<double, double>> acts;
    acts.clear();
    for (int i = 0; i < 6; i++)
    {
      double actionL, actionR;
      actionL = boost::python::extract<double>(actions[i][0]);
      actionR = boost::python::extract<double>(actions[i][1]);
      acts.push_back(std::tuple<double, double>(actionL, actionR));
    }
    world->step(Config::World().getDeltaTime(), acts);
    boost::python::list res;
    for (double feature : world->getState())
    {
      res.append(feature);
    }
    return res;
  }
  bool getDone() { return world->done; }
};

BOOST_PYTHON_MODULE(librobosim)
{
  class_<RoboSim>("RoboSim")
      .def("del", &RoboSim::del)
      .def("step", &RoboSim::step)
      .def("getDone", &RoboSim::getDone);
}