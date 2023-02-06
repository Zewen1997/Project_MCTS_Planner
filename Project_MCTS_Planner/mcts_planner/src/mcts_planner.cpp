#include "mcts_planner.h"

MCTSPlanner::MCTSPlanner(){mcts_data_handler_ = std::make_shared<MCTSDataHandler>();
mcts_exp3_ = std::make_shared<Exp3>();
}

void MCTSPlanner::processInit()
{   
    // MCTS Initialization
    
   
    // mcts_solver_ = MCTS<MCTSState, ActionType>();
    /*
    error: 1. mcts_state_ = MCTSState(mcts_datahandler);
           2. MCTSState mct_state(mcts_datahandler);
              mcts_state_ = mcts_state;
    */
    
    mcts_state_.mcts_data_handler_ = mcts_data_handler_;
    mcts_solver_.mcts_data_handler_ = mcts_data_handler_;
    mcts_solver_.mcts_exp3_ = mcts_exp3_;


    

    // MCTS Parameters Initialization
    mcts_solver_.uct_k_            = 1;
    mcts_solver_.max_millis_       = 10000;
    mcts_solver_.max_iterations_   = 1000;
    mcts_solver_.simulation_depth_ = 10;
    mcts_solver_.max_tree_depth_   = 12;
    mcts_solver_.to_be_normalized_score_ = {FLT_EPSILON}; 
    mcts_exp3_->init(0.3, 4);
}


void MCTSPlanner::processRun()
{  

   // create current state
   mcts_state_.createStartState(mcts_data_handler_->getEnvironmentState());
   
   // search on current state
   mcts_solver_.search(mcts_state_);

   // get the best action
   // mcts_actions_ = mcts_solver_.getMostVisitedActions();
   mcts_actions_ = mcts_solver_.getBestUCTActions();

   // update actions to decision data
   if (mcts_actions_.size() > 0)
   {  for(int i = 0; i < mcts_actions_.size(); i++){std::cout<<mcts_actions_[i]<<std::endl;}

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

  mcts_data_handler_->setDecisions(accelerations);

}

void MCTSPlanner::setDecisions(std::vector<float>& decisions)
{
  mcts_data_handler_->setDecisions(decisions);
}


std::vector<float> MCTSPlanner::getDecisions()
{
  return mcts_data_handler_->getDecisions();
}

void MCTSPlanner::setEnvironmentState(EnvironmentState& environment_state)
{
  mcts_data_handler_->setEnvironmentState(environment_state);
}

EnvironmentState MCTSPlanner::getEnvironmentState()
{
  return mcts_data_handler_->getEnvironmentState();
}

void MCTSPlanner::setReferencePathEgo(std::vector<float>& reference_path_ego)
{   
    mcts_data_handler_->setReferencePathEgo(reference_path_ego);

}

std::vector<float> MCTSPlanner::getReferencePathEgo()
{
  return mcts_data_handler_->getReferencePathEgo();

}

void MCTSPlanner::setWaypointEgo(std::vector<Waypoint>& waypoint_ego)
{   
    mcts_data_handler_->setWaypointEgo(waypoint_ego);

}

std::vector<Waypoint> MCTSPlanner::getWaypointEgo()
{
  return mcts_data_handler_->getWaypointEgo();

}

void MCTSPlanner::setReferencePathVehicles(std::unordered_map<int, S>& reference_path_vehicles)
{   
    mcts_data_handler_->setReferencePathVehicles(reference_path_vehicles);

}

std::unordered_map<int, S> MCTSPlanner::getReferencePathVehicles()
{
  return mcts_data_handler_->getReferencePathVehicles();

}

void MCTSPlanner::setWaypointVehicles(std::unordered_map<int, Route>& route_vehicles)
{   
    mcts_data_handler_->setWaypointVehicles(route_vehicles);

}

std::unordered_map<int, Route> MCTSPlanner::getWaypointVehicles()
{
  return mcts_data_handler_->getWaypointVehicles();
}
