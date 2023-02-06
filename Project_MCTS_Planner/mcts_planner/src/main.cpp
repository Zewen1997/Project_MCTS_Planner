#include <iostream>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


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
            .def(pybind11::init<const std::vector<float>, const float, const float, const float, const float, const float, const float, const int>())
            .def("print", &ObjectState::print);

    pybind11::class_<StatesStatus>(m, "StatesStatus")
            .def(pybind11::init<const bool, const bool>())
            .def("print", &StatesStatus::print);

    pybind11::class_<EnvironmentState>(m, "EnvironmentState")
            .def(pybind11::init<const EgoState&, const std::vector<ObjectState>&, const StatesStatus&>())
            .def("print", &EnvironmentState::print);


    pybind11::class_<Waypoint>(m, "Waypoint")
            .def(pybind11::init<const float, const float, const float, const float, const float>());

    pybind11::class_<SortVehicle>(m, "SortVehicle")
            .def(pybind11::init<const ObjectState&, const float>());

    pybind11::class_<Route>(m, "Route")
            .def(pybind11::init<std::unordered_map <int, std::vector<Waypoint>>& >());

    pybind11::class_<S>(m, "S")
            .def(pybind11::init<std::unordered_map <int, std::vector<float>>& >());

    /*
    pybind11::class_<MCTSDataHandler, std::shared_ptr<MCTSDataHandler>>(m, "MCTSDataHandler")
            .def(pybind11::init<>())
            .def("setEnvironmentState", &MCTSDataHandler::setEnvironmentState)
            .def("getEnvironmentState", &MCTSDataHandler::getEnvironmentState);
    */

    pybind11::class_<MCTSState, std::shared_ptr<MCTSState>>(m, "MCTSState")
            .def(pybind11::init<>())
            .def("createStartState",&MCTSState::createStartState)
            .def("calculateNextEgoState", &MCTSState::calculateNextEgoState)
            .def("calculateNextObjectsState", &MCTSState::calculateNextObjectsState)
            .def("calculateNextStatus", &MCTSState::calculateNextStatus)
            .def("setReferencePathEgo", &MCTSState::setReferencePathEgo)
            .def("getReferencePathEgo", &MCTSState::getReferencePathEgo)
            .def("setWaypointEgo", &MCTSState::setWaypointEgo)
            .def("getWaypointEgo", &MCTSState::getWaypointEgo)
            .def("setReferencePathVehicles", &MCTSState::setReferencePathVehicles)
            .def("getReferencePathVehicles", &MCTSState::getReferencePathVehicles)
            .def("setWaypointVehicles", &MCTSState::setWaypointVehicles)
            .def("getWaypointVehicles", &MCTSState::getWaypointVehicles);

    
    pybind11::class_<MCTSPlanner, std::shared_ptr<MCTSPlanner>>(m, "MCTSPlanner")
            .def(pybind11::init<>())
            .def("processInit", &MCTSPlanner::processInit)
            .def("processRun", &MCTSPlanner::processRun)
            .def("setDecisions", &MCTSPlanner::setDecisions)
            .def("getDecisions", &MCTSPlanner::getDecisions)
            .def("setEnvironmentState", &MCTSPlanner::setEnvironmentState)
            .def("getEnvironmentState", &MCTSPlanner::getEnvironmentState)
            .def("setReferencePathEgo", &MCTSPlanner::setReferencePathEgo)
            .def("getReferencePathEgo", &MCTSPlanner::getReferencePathEgo)
            .def("setWaypointEgo", &MCTSPlanner::setWaypointEgo)
            .def("getWaypointEgo", &MCTSPlanner::getWaypointEgo)
            .def("setReferencePathVehicles", &MCTSPlanner::setReferencePathVehicles)
            .def("getReferencePathVehicles", &MCTSPlanner::getReferencePathVehicles)
            .def("setWaypointVehicles", &MCTSPlanner::setWaypointVehicles)
            .def("getWaypointVehicles", &MCTSPlanner::getWaypointVehicles);

    

}
