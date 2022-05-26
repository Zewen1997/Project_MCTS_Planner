#ifndef MCTS_INTERFACE_H
#define MCTS_INTERFACE_H

#include <iostream>
#include <unordered_map>
#include "mcts_utils.h"


class MCTSInterface
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

public:
    MCTSInterface();

    void setEnvironmentState(const EnvironmentState& mcts_state);
    EnvironmentState& getEnvironmentState();

    // TODO update decision
    void setDecisions(const std::vector<float>& decisions);
    std::vector<float> getDecisions();

    void setReferencePathEgo(const std::vector<float>& reference_path_ego);
    std::vector<float> getReferencePathEgo();

    void setWaypointEgo(const std::vector<Waypoint>& waypoint_ego);
    std::vector<Waypoint> getWaypointEgo();

};



#endif // MCTS_INTERFACE_H