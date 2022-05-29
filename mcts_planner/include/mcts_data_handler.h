#ifndef MCTS_DATA_HANDLER_H
#define MCTS_DATA_HANDLER_H

#include <iostream>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <memory>
#include "mcts_utils.h"


class MCTSDataHandler
{

private:
    std::string        interface_name_;
    std::vector<float> decisions_;
    std::vector<float> reference_path_ego_;
    std::vector<Waypoint> waypoint_ego_;
    std::unordered_map<int, std::vector<float>> reference_path_vehicle_;
    std::unordered_map<int, Route> route_vehicle_;
    EnvironmentState   environment_state_;
    bool use_exp3_;
    int number_vehicle = 5;


public:
    MCTSDataHandler();

    // ---------------------------- Interface functions ------------------------ //
    void setEnvironmentState(const EnvironmentState& mcts_state);
    EnvironmentState& getEnvironmentState();

    // TODO update decision
    void setDecisions(const std::vector<float>& decisions);
    std::vector<float>& getDecisions();

    void setReferencePathEgo(const std::vector<float>& reference_path_ego);
    std::vector<float> getReferencePathEgo();

    void setWaypointEgo(const std::vector<Waypoint>& waypoint_ego);
    std::vector<Waypoint> getWaypointEgo();

    // ---------------------------- Data process ------------------------------- //
    float distance(float x1, float x2, float y1, float y2);
    std::vector<SortVehicle> getNearstVehicles(EgoState& ego, std::vector<ObjectState>& vehicles);

};



#endif 