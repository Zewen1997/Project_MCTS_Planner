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
    //std::shared_ptr<MCTSDataHandler>   mcts_data_handler_;
    MCTSDataHandler mcts_data_handler_;
    
public:
    MCTSPlanner();

    void                    processInit();
    void                    processRun();
    void                    updateDecisionsVector();
    void                    setDecisions(std::vector<float>& decisions);
    std::vector<float>      getDecisions();
    void                    setReferencePathEgo(std::vector<float>& reference_path_ego);
    std::vector<float>      getReferencePathEgo();
};

#endif // MCTS_PLANNER_H