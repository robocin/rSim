#include <iostream>
#include <utility>
#include <cstdint>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "vssworld.h"
#include "sslworld.h"

namespace py = pybind11;
using vd = std::vector<double>;
using vvd = std::vector<std::vector<double>>;

struct VSS {
    VSS(int fieldType, int nRobotsBlue, int nRobotsYellow, int timeStep_ms,
        const vd &ballPos, const vvd &blueRobotsPos, const vvd &yellowRobotsPos) : m_fieldType(fieldType),
                                                                                   m_nRobotsBlue(nRobotsBlue),
                                                                                   m_nRobotsYellow(nRobotsYellow),
                                                                                   m_timeStep_ms(timeStep_ms) {
        m_world = new VSSWorld(m_fieldType, m_nRobotsBlue, m_nRobotsYellow, m_timeStep_ms / 1000.0,
                               ballPos, blueRobotsPos, yellowRobotsPos);
    }

    ~VSS() { delete m_world; }

    void step(vvd actions) const { m_world->step(std::move(actions)); }

    vd getState() const { return m_world->getState(); }

    void reset(const vd &ballPos, const vvd &blueRobotsPos, const vvd &yellowRobotsPos) {
        delete m_world;
        m_world = new VSSWorld(m_fieldType, m_nRobotsBlue, m_nRobotsYellow, m_timeStep_ms / 1000.0,
                               ballPos, blueRobotsPos, yellowRobotsPos);
    }

    std::unordered_map<std::string, double> getFieldParams() const { return m_world->getFieldParams(); }

    VSSWorld *m_world;
    int m_fieldType, m_nRobotsBlue, m_nRobotsYellow, m_timeStep_ms;
};

struct SSL {
    SSL(int fieldType, int nRobotsBlue, int nRobotsYellow, int timeStep_ms,
        const vd &ballPos, const vvd &blueRobotsPos, const vvd &yellowRobotsPos) : m_fieldType(fieldType),
                                                                                   m_nRobotsBlue(nRobotsBlue),
                                                                                   m_nRobotsYellow(nRobotsYellow),
                                                                                   m_timeStep_ms(timeStep_ms) {
        m_world = new SSLWorld(m_fieldType, m_nRobotsBlue, m_nRobotsYellow, m_timeStep_ms / 1000.0,
                               ballPos, blueRobotsPos, yellowRobotsPos);
    }

    ~SSL() { delete m_world; }

    void step(vvd actions) const { m_world->step(std::move(actions)); }

    vd getState() const { return m_world->getState(); }

    void reset(const vd &ballPos, const vvd &blueRobotsPos, const vvd &yellowRobotsPos) {
        delete m_world;
        m_world = new SSLWorld(m_fieldType, m_nRobotsBlue, m_nRobotsYellow, m_timeStep_ms / 1000.0,
                               ballPos, blueRobotsPos, yellowRobotsPos);
    }

    std::unordered_map<std::string, double> getFieldParams() const { return m_world->getFieldParams(); }

    SSLWorld *m_world;
    int m_fieldType, m_nRobotsBlue, m_nRobotsYellow, m_timeStep_ms;
};


PYBIND11_MODULE(_robosim, m) {
    py::class_<VSS>(m, "VSS")
            .def(py::init<int, int, int, int, vd, vvd, vvd>())
            .def("step", &VSS::step)
            .def("get_state", &VSS::getState)
            .def("reset", &VSS::reset)
            .def("get_field_params", &VSS::getFieldParams);

    py::class_<SSL>(m, "SSL")
            .def(py::init<int, int, int, int, vd, vvd, vvd>())
            .def("step", &SSL::step)
            .def("get_state", &SSL::getState)
            .def("reset", &SSL::reset)
            .def("get_field_params", &SSL::getFieldParams);
}


