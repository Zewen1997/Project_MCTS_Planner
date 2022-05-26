#include "mcts_interface.h"


MCTSInterface::MCTSInterface()
{
    use_exp3_ = false;
}

void MCTSInterface::setEnvironmentState(const EnvironmentState& environment_state)
{
    environment_state_ = environment_state;
    environment_state_.print();
}

EnvironmentState& MCTSInterface::getEnvironmentState()
{
    return environment_state_;
}

void MCTSInterface::setDecisions(const std::vector<float>& decisions)
{
    decisions_ = decisions;
}

std::vector<float> MCTSInterface::getDecisions()
{   
    return decisions_;
}


void MCTSInterface::setReferencePathEgo(const std::vector<float>& reference_path_ego)
{
    reference_path_ego_ = reference_path_ego;
}

std::vector<float> MCTSInterface::getReferencePathEgo()
{
    return reference_path_ego_;
}

void MCTSInterface::setWaypointEgo(const std::vector<Waypoint>& waypoint_ego)
{
    waypoint_ego_ = waypoint_ego;
}

std::vector<Waypoint> MCTSInterface::getWaypointEgo()
{
    return waypoint_ego_;
}

