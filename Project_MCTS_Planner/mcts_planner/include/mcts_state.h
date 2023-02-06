#ifndef MCTS_STATE_H
#define MCTS_STATE_H

#include "mcts_utils.h"
#include "mcts_data_handler.h"


class MCTSState
{
 private:
  EnvironmentState                   state_;  // Define MCTS Planner State Space
  // MCTSDataHandler  mcts_data_handler_;

 public:
  // copy and assignment operators should perform a DEEP clone of the given state
  std::shared_ptr<MCTSDataHandler>   mcts_data_handler_;

  MCTSState(); 
  // MCTSState(std::shared_ptr<MCTSDataHandler> mcts_data_handler);
  MCTSState(const MCTSState& other, std::shared_ptr<MCTSDataHandler> mcts_data_handler);
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
  //bool getRandomAction(ActionType& action) const;
  ActionType getRandomAction() ;

  // evaluate this state and return a vector of rewards (for each agent)
  const std::vector<float> evaluateRewards(ActionType &action) const;

  // return state as string (for debug purposes)
  std::string printState() const;

  /* -------- Model related functions -------- */
  EgoState                                    calculateNextEgoState(EnvironmentState const& state, ActionType const& action);
  std::vector<ObjectState>                    calculateNextObjectsState(EnvironmentState const& state, const int depth);
  StatesStatus                                calculateNextStatus(EgoState const& next_ego_state, std::vector<ObjectState> const& next_objects_state) const;
  float                                       getAcceleration(const ActionType action) const;
  std::string                                 getAccelerationType(const ActionType action) const;
  void                                        setReferencePathEgo(std::vector<float>& reference_path_ego);
  std::vector<float>                          getReferencePathEgo();
  void                                        setWaypointEgo(std::vector<Waypoint>& waypoint_ego);
  std::vector<Waypoint>                       getWaypointEgo();
  void                                        setReferencePathVehicles(std::unordered_map<int, S>& reference_path_vehicles);
  std::unordered_map<int, S>                  getReferencePathVehicles(); 
  void                                        setWaypointVehicles(std::unordered_map<int, Route>& route_vehicles);
  std::unordered_map<int, Route>              getWaypointVehicles();
  //void                                        setDataHandler(std::shared_ptr<MCTSDataHandler>& mcts_data_handler);
  //std::shared_ptr<MCTSDataHandler>            getDataHandler() const;

};

#endif