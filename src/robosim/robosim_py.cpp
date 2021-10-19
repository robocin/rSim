#include <iostream>
#include <pybind11/pybind11.h>
// #include "sslworld.h"

namespace py = pybind11;

struct VSS {
    VSS(int fieldType, int nRobotsBlue, int nRobotsYellow) : fieldType(fieldType), nRobotsBlue(nRobotsBlue), nRobotsYellow(nRobotsYellow) {}
    ~VSS() {}
    void step() { std::cout << fieldType << " step!\n" ;}
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
        .def(py::init<int, int, int>())
        .def("step", &VSS::step);
}
