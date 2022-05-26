#ifndef MCTS_STATE_H
#define MCTS_STATE_H

#include "mcts_utils.h"
#include "mcts_interface.h"

class MCTSState
{
 private:
  EnvironmentState state_;  // Define MCTS Planner State Space
  MCTSInterface mcts_interface_; 

 public:
  // copy and assignment operators should perform a DEEP clone of the given state
  MCTSState();
  MCTSState(const MCTSState& other);
  MCTSState& operator=(const MCTSState& other);

  /* -------- Setters and Getters -------- */
  const EnvironmentState& getState() const;

  /* -------- Interface related functions -------- */
  // generate start state
  void createStartState(const EnvironmentState& start_state);

  // whether or not this state is terminal (reached end)
  bool isTerminal() const;

  //  agent id (zero-based) for agent who is about to make a decision
  int agentId() const;

  // apply action to state
  void applyAction(const ActionType& action, int current_depth);

  // return possible actions from this state
  void getActions(std::vector<ActionType>& actions) const;

  // get a random action, return false if no actions found
  bool getRandomAction(ActionType& action) const;

  // evaluate this state and return a vector of rewards (for each agent)
  const std::vector<float> evaluateRewards() const;

  // return state as string (for debug purposes)
  std::string printState() const;

  /* -------- Model related functions -------- */
  EgoState                 calculateNextEgoState(EnvironmentState const& state, ActionType const& action);
  std::vector<ObjectState> calculateNextObjectsState(EnvironmentState const& state, const int depth, std::unordered_map<int, std::vector<float>> const reference_path, std::unordered_map<int, Route> route_vehicle) const;
  StatesStatus             calculateNextStatus(EgoState const& next_ego_state, std::vector<ObjectState> const& next_objects_state) const;
  float                    getAcceleration(const ActionType action) const;
  std::string              getAccelerationType(const ActionType action) const;

};

#endif