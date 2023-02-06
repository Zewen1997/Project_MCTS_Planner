#ifndef    MCTS_PLANNER_H
#define    MCTS_PLANNER_H

#include "mcts_data_handler.h"
#include "mcts_state.h"
#include "mcts_solver.h"

class MCTSPlanner
{
private:

    MCTSState                          mcts_state_;
    std::vector<ActionType>            mcts_actions_;
    MCTS<MCTSState, ActionType>        mcts_solver_;
    std::shared_ptr<MCTSDataHandler>   mcts_data_handler_;
    std::shared_ptr<Exp3>              mcts_exp3_;
    // MCTSDataHandler mcts_data_handler_;
    
public:
    MCTSPlanner();

    void                                        processInit();
    void                                        processRun();
    void                                        updateDecisionsVector();
    void                                        setDecisions(std::vector<float>& decisions);
    std::vector<float>                          getDecisions();
    void                                        setEnvironmentState(EnvironmentState& environment_state);
    EnvironmentState                            getEnvironmentState();
    void                                        setReferencePathEgo(std::vector<float>& reference_path_ego);
    std::vector<float>                          getReferencePathEgo();
    void                                        setWaypointEgo(std::vector<Waypoint>& waypoint_ego);
    std::vector<Waypoint>                       getWaypointEgo();
    void                                        setReferencePathVehicles(std::unordered_map<int, S>& reference_path_vehicles);
    std::unordered_map<int, S>                  getReferencePathVehicles(); 
    void                                        setWaypointVehicles(std::unordered_map<int, Route>& route_vehicles);
    std::unordered_map<int, Route>              getWaypointVehicles();
  
};

#endif // MCTS_PLANNER_H
