#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "vssworld.h"

namespace py = pybind11;
using vd = std::vector<double>;
using vvd = std::vector<std::vector<double>>;

struct VSS {
    VSS(int fieldType, int nRobotsBlue, int nRobotsYellow, int timeStep_ms,
        const vd& ballPos, const vvd& blueRobotsPos, const vvd& yellowRobotsPos) {
//        m_world = new VSSWorld(fieldType, nRobotsBlue, nRobotsYellow, timeStep_ms / 1000.0,
//                             ballPos, blueRobotsPos, yellowRobotsPos);
    }
    ~VSS() { delete m_world;}
    std::vector<double> getState() { return m_world->getState(); }
    void step() { std::cout << fieldType << " step!\n" ;}
    VSSWorld *m_world;
    int fieldType;
    int nRobotsBlue;
    int nRobotsYellow;
};

struct SSL {
    SSL(int fieldType, int nRobotsBlue, int nRobotsYellow) : fieldType(fieldType), nRobotsBlue(nRobotsBlue), nRobotsYellow(nRobotsYellow) {}
    ~SSL() {}
    void step() { std::cout << fieldType << " step!\n" ;}
    int fieldType;
    int nRobotsBlue;
    int nRobotsYellow;
};

PYBIND11_MODULE(robosim, m) {
    py::class_<SSL>(m, "SSL")
        .def(py::init<int, int, int>())
        .def("step", &SSL::step);
    py::class_<VSS>(m, "VSS")
        .def(py::init<int, int, int, int, vd, vvd, vvd>())
        .def("get_state", &VSS::getState);
}
