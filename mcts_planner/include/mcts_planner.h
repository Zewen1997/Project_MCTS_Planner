#ifndef    MCTS_PLANNER_H
#define    MCTS_PLANNER_H

#include "mcts_interface.h"
#include "mcts_state.h"
#include "mcts_solver.h"

class MCTSPlanner
{
private:
    MCTSInterface               mcts_interface_;

    MCTSState                   mcts_state_;
    std::vector<ActionType>     mcts_actions_;
    MCTS<MCTSState, ActionType> mcts_solver_;

public:
    MCTSPlanner();

    MCTSInterface& interface();
    
    void    processInit();
    void    processRun();
    void    updateDecisionsVector();

};

#endif // MCTS_PLANNER_H