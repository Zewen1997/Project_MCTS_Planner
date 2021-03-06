#include "mcts_planner.h"

MCTSPlanner::MCTSPlanner(){mcts_data_handler_ = MCTSDataHandler();}

void MCTSPlanner::processInit()
{   
    // MCTS Initialization
    
    mcts_solver_ = MCTS<MCTSState, ActionType>();
    //mcts_data_handler_ = std::make_shared<MCTSDataHandler>();
    mcts_state_  = MCTSState();

    // MCTS Parameters Initialization
    mcts_solver_.uct_k_            = 2;
    mcts_solver_.max_millis_       = 500;
    mcts_solver_.max_iterations_   = 50000;
    mcts_solver_.simulation_depth_ = 10;
    mcts_solver_.max_tree_depth_   = 10;

}


void MCTSPlanner::processRun()
{
   // create current state
   mcts_state_.createStartState(mcts_data_handler_.getEnvironmentState());
   
   // search on current state
   mcts_solver_.search(mcts_state_);

   // get the best action
   mcts_actions_ = mcts_solver_.getMostVisitedActions();

   // update actions to decision data
   if (mcts_actions_.size() > 0)
   {
      std::cout << "---------------------------- " << std::endl;
      std::cout << "total actions: " << mcts_actions_.size() << std::endl;
      std::cout << "mcts action: " << mcts_state_.getAccelerationType(mcts_actions_.front()) << std::endl;
      std::cout << "total iterations: " << mcts_solver_.getIterations() << std::endl;
      std::cout << "---------------------------- " << std::endl;
      updateDecisionsVector();
   }
}

void MCTSPlanner::updateDecisionsVector()
{
  std::vector<float>       accelerations;
  std::vector<std::string> accelerations_type;
  std::vector<float>       planning_time{0.0f};

  for (int i = 0; i < mcts_actions_.size(); i++)
  {
    float     accleration       = mcts_state_.getAcceleration(mcts_actions_.at(i));
    std::string acceleration_type = mcts_state_.getAccelerationType(mcts_actions_.at(i));

    accelerations.push_back(accleration);
    accelerations_type.push_back(acceleration_type);
    planning_time.push_back(1.0f); // time_step, TODO use a parameter
  }

//    mcts_interface_.setDecisions(accelerations);
    std::vector<float>       test_accelarations(5, 2.0);
    mcts_data_handler_.setDecisions(test_accelarations);

}

void MCTSPlanner::setDecisions(std::vector<float>& decisions)
{
  mcts_data_handler_.setDecisions(decisions);
}


std::vector<float> MCTSPlanner::getDecisions()
{
  return mcts_data_handler_.getDecisions();
}

void MCTSPlanner::setReferencePathEgo(std::vector<float>& reference_path_ego)
{   
    mcts_data_handler_.setReferencePathEgo(reference_path_ego);

}

std::vector<float> MCTSPlanner::getReferencePathEgo()
{
  return mcts_data_handler_.getReferencePathEgo();
}