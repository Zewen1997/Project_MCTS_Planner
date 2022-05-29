#include <iostream>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


#include "mcts_data_handler.h"
#include "mcts_planner.h"
#include "mcts_data_handler.h"
#include "ei/2dtypes.hpp"
#include "ei/2dintersection.hpp"

PYBIND11_MODULE(MCTS, m) {
    pybind11::enum_<ActionType>(m, "ActionType")
            .value("DEC", DEC)
            .value("CON", CON)
            .value("ACC", ACC)
            .value("EMERGENCY_BRAKE", EMERGENCY_BRAKE)
            .export_values();

    pybind11::class_<EgoState>(m, "EgoState")
            .def(pybind11::init<const float, const float, const float, const float, const float, const float, const float>())
            .def("print", &EgoState::print);

    pybind11::class_<ObjectState>(m, "ObjectState")
            .def(pybind11::init<const std::vector<float>, const float, const float, const float, const float, const float, const float, const int>());

    pybind11::class_<StatesStatus>(m, "StatesStatus")
            .def(pybind11::init<const bool, const bool>());

    pybind11::class_<EnvironmentState>(m, "EnvironmentState")
            .def(pybind11::init<const EgoState&, const std::vector<ObjectState>&, const StatesStatus&>())
            .def("print", &EnvironmentState::print);

    pybind11::class_<Waypoint>(m, "Waypoint")
            .def(pybind11::init<const float, const float, const float, const float, const float>());

    pybind11::class_<SortVehicle>(m, "SortVehicle")
            .def(pybind11::init<const ObjectState&, const float>());

    pybind11::class_<Route>(m, "Route")
            .def(pybind11::init<std::unordered_map <int, std::vector<Waypoint>>& >());

    pybind11::class_<MCTSDataHandler>(m, "MCTSDataHandler")
            .def(pybind11::init<>())
            .def("setEnvironmentState", &MCTSDataHandler::setEnvironmentState)
            .def("getEnvironmentState", &MCTSDataHandler::getEnvironmentState);

    pybind11::class_<MCTSState>(m, "MCTSState")
            .def(pybind11::init<>())
            .def("createStartState",&MCTSState::createStartState)
            .def("calculateNextEgoState", &MCTSState::calculateNextEgoState)
            .def("setReferencePathEgo", &MCTSState::setReferencePathEgo)
            .def("getReferencePathEgo", &MCTSState::getReferencePathEgo)
            .def("setWaypointEgo", &MCTSState::setWaypointEgo)
            .def("getWaypointEgo", &MCTSState::getWaypointEgo);


    pybind11::class_<MCTSPlanner>(m, "MCTSPlanner")
            .def(pybind11::init<>())
            .def("processInit", &MCTSPlanner::processInit)
            .def("processRun", &MCTSPlanner::processRun)
            .def("setDecisions", &MCTSPlanner::setDecisions)
            .def("getDecisions", &MCTSPlanner::getDecisions)
            .def("setReferencePathEgo", &MCTSPlanner::setReferencePathEgo)
            .def("getReferencePathEgo", &MCTSPlanner::getReferencePathEgo);



}
