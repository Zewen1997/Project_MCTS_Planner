#include "mcts_state.h"
#include "ei/2dtypes.hpp"
#include "ei/2dintersection.hpp"
#include "ei/vector.hpp"
#include <algorithm>
#include <cmath>
#include <cassert>
#include <ctime>
#include "mcts_data_handler.h"


using namespace ei;

MCTSState::MCTSState(){mcts_data_handler_ = MCTSDataHandler();}
//MCTSState::MCTSState(std::shared_ptr<MCTSDataHandler> mcts_data_handler){mcts_data_handler_ = mcts_data_handler;}


MCTSState::MCTSState(const MCTSState &other)
{
  state_ = other.getState();
}

MCTSState &MCTSState::operator=(const MCTSState &other)
{
  state_ = other.getState();
}

const EnvironmentState &MCTSState::getState() const
{
  return state_;
}


void MCTSState::createStartState(const EnvironmentState& start_state)
{
  // update inside state_
  state_ = start_state;
}

bool MCTSState::isTerminal() const
{
    return state_.getStatesStatus().is_collied;
}

int MCTSState::agentId() const
{
  // only have ego as a agent
  return 0;
}

void MCTSState::applyAction(const ActionType &action, int current_depth)
{
  // current state to next state for the given action

//   // 1. Update next ego state
//   DEgoState next_ego_state = this->calculateNextEgoState(state_, action);

//   // 2. Update next objects state
//   std::vector<DObjectState> next_objects_state = this->calculateNextObjectsState(state_, current_depth);

//   // 3. Update status between ego and objects
//   DStatesStatus next_states_status = this->calculateNextStatus(next_ego_state, next_objects_state);
//   state_.setDespotState(next_ego_state, next_objects_state, next_states_status);
}

void MCTSState::getActions(std::vector<ActionType> &actions) const
{
  int number_actions = 3;
  // Get all possible actions
  for (long code = 0; code < number_actions; code++)
  {
    switch (code)
    {
      case 0:
        actions.push_back(ActionType::DEC);
        break;
      case 1:
        actions.push_back(ActionType::CON);
        break;
      case 2:
        actions.push_back(ActionType::ACC);
        break;
      default:
        break;
    }
  }
}

bool MCTSState::getRandomAction(ActionType &action) const
{
  std::vector<ActionType> actions;
  this->getActions(actions);

  // randomize the order
  std::random_shuffle(actions.begin(), actions.end());
  action = actions.front();
}


const std::vector<float> MCTSState::evaluateRewards() const
{
  // evalutate rewards based next state (state_ here is already a next state)   
  EgoState            next_ego_state     = state_.getEgoState();
  StatesStatus        next_states_status = state_.getStatesStatus();
  std::vector<float> reward(1, 0.f);

  // Parameters TODO yaml
  float collision_reward     = -5000.f;
  float keep_velocity_reward = -100.f;
  float spped_limit_reward   = -1000.f;
  float action_reward        = -140.f;

  // 1. collision reward
  if (next_states_status.is_collied)
  {
    reward.front() += collision_reward;
  }

  // 2. Velocity reward
  // Get desired velocity
  float velocity_desired = 10;
  if (velocity_desired - next_ego_state.velocity >= 0.f)
  {
    reward.front() += keep_velocity_reward * std::fabs(velocity_desired - next_ego_state.velocity);
  }
  else
  {
    reward.front() += spped_limit_reward * std::fabs(velocity_desired - next_ego_state.velocity);
  }

  return reward;
}


std::string MCTSState::printState() const
{
  std::cout << std::endl;
  std::cout << "  Ego_station:\t\t " << state_.getEgoState().ego_station << std::endl;
  std::cout << "  Position x:\t\t " << state_.getEgoState().x << std::endl;
  std::cout << "  Position y:\t\t " << state_.getEgoState().y << std::endl;
  std::cout << "  Orientation theta:\t " << state_.getEgoState().psi << std::endl;
  std::cout << "  Velocity v:\t\t " << state_.getEgoState().velocity << std::endl;
  std::cout << "  Status: " << (state_.getStatesStatus().is_collied ? "Collided." : "Not collided.") << std::endl;

  std::cout << " size " << state_.getObjectState().size() << "\n";
  for (int i = 0; i < state_.getObjectState().size(); ++i)
  {
    std::cout << "  ObjectsPosition x:\t\t " << state_.getObjectState().at(i).x << std::endl;
    std::cout << "  ObjectsPosition y:\t\t " << state_.getObjectState().at(i).y << std::endl;
  }
  std::cout << std::endl;
}


EgoState MCTSState::calculateNextEgoState(EnvironmentState const& state, ActionType const& action) 
{ 
  EgoState current_ego_state = state.getEgoState();
  EgoState next_ego_state;
  next_ego_state = current_ego_state;
  Waypoint waypoint_right;
  Waypoint waypoint_left;
  std::vector<float> reference_path = mcts_data_handler_.getReferencePathEgo();

  std::cout<<"reference_path_size:"<<std::endl;
  std::cout<<reference_path.size()<<std::endl;


  std::vector<Waypoint> waypoint_ego = mcts_data_handler_.getWaypointEgo();
  std::cout<<"waypoint_ego_size:"<<std::endl;
  std::cout<<waypoint_ego.size()<<std::endl;

  float   time_step = 0.1f;  // TODO time_step as a parameter
  float   weight;
  
  // Get acceleration value
  const float acceleration = getAcceleration(action);
  next_ego_state.ego_station = current_ego_state.ego_station + current_ego_state.ego_station * time_step + 0.5 * acceleration * pow(time_step, 2);
  std::cout<<next_ego_state.ego_station<<std::endl;
  int index = std::distance(reference_path.begin(), std::lower_bound(reference_path.begin(), reference_path.end(), next_ego_state.ego_station));
  std::cout<<"index: "<<index<<std::endl;

  if (index >= reference_path.size()){
    index = reference_path.size() - 1;
  }
  
  if (index != 0 ){
    
    waypoint_right = waypoint_ego[index];
    waypoint_left = waypoint_ego[index - 1];
    weight = (next_ego_state.ego_station - waypoint_left.station) / (waypoint_right.station - waypoint_left.station);
    next_ego_state.x = weight * (waypoint_right.x - waypoint_left.x) + waypoint_left.x;
    next_ego_state.y = weight * (waypoint_right.y - waypoint_left.y) + waypoint_left.y;
    next_ego_state.psi = weight * (waypoint_right.psi - waypoint_left.psi) + waypoint_left.psi;
    next_ego_state.velocity = current_ego_state.velocity + acceleration * time_step;
    
  }
  
  
  
  
  return next_ego_state;
}

std::vector<ObjectState> MCTSState::calculateNextObjectsState(EnvironmentState const& state, const int depth, std::unordered_map<int, std::vector<float>> reference_path, std::unordered_map<int, Route> route_vehicle)
{
  std::vector<ObjectState>  next_objects_state;
  std::vector<ObjectState> sorted_objects_state;
  Waypoint waypoint_right;
  Waypoint waypoint_left;
  float time_step = 1.0f;  // TODO time_step as a parameter
  float weight;
  int k; // k-1 cars will use waypoint to update state, remaining cars use constant speed

  if (!state.getObjectState().empty())
  {
    // calculate each object state of next time step
    EgoState current_ego_state = state.getEgoState();
    std::vector<ObjectState> current_objects_state = state.getObjectState();
    std::vector<SortVehicle> sorted_vehicles = mcts_data_handler_.getNearstVehicles(current_ego_state, current_objects_state);
    
    
    
    for(int i = 0; i < sorted_vehicles.size(); i++)
    {
      ObjectState object_state = sorted_vehicles[i].object_state_;
      sorted_objects_state.emplace_back(object_state);
      
    }

    for (auto const &object : sorted_objects_state)
    { 
      ObjectState next_object_state;
      std::vector <float> station = object.object_station;
      if (k <= 1){
        for(float &x: station)
        x += object.velocity * time_step;
        next_object_state.object_station = station;
        srand(time(0));
        int random_route = rand()%(route_vehicle[object.id].route.size());
        int index = std::distance(reference_path[object.id].begin(), std::lower_bound(reference_path[object.id].begin(), reference_path[object.id].end(), next_object_state.object_station[random_route]));
        if(index != 0){
          waypoint_right = route_vehicle[object.id].route[random_route][index];
          waypoint_left = route_vehicle[object.id].route[random_route][index - 1];
          weight = (next_object_state.object_station[random_route] - waypoint_left.station) / (waypoint_right.station - waypoint_left.station);
          next_object_state.x = weight * (waypoint_right.x - waypoint_left.x) + waypoint_left.x;
          next_object_state.y = weight * (waypoint_right.y - waypoint_left.y) + waypoint_left.y;
          next_object_state.psi = weight * (waypoint_right.psi - waypoint_left.psi) + waypoint_left.psi;

        }
      }
      next_object_state.psi = object.psi;
      next_object_state.velocity  = object.velocity;
      next_object_state.x   = object.x + time_step * (next_object_state.velocity + object.velocity) * std::cos(object.psi) / 2;
      next_object_state.y   = object.y + time_step * (next_object_state.velocity + object.velocity) * std::sin(object.psi) / 2;
      next_objects_state.emplace_back(next_object_state);
    }
  }

  return next_objects_state;
}

StatesStatus MCTSState::calculateNextStatus(EgoState const& next_ego_state, std::vector<ObjectState> const& next_objects_state) const
{
  StatesStatus state_status;

  // check collision with shapely polygon
  bool is_collied = false;
 
  Rect2D rect_ego(Vec2(next_ego_state.x - next_ego_state.length / 2.0f * cos(next_ego_state.psi * 180 / PI) + next_ego_state.width / 2.0f * sin(next_ego_state.psi * 180 / PI), 
                       next_ego_state.y - next_ego_state.length / 2.0f * sin(next_ego_state.psi * 180 / PI) - next_ego_state.width / 2.0f * cos(next_ego_state.psi * 180 / PI)), 
                  Vec2(next_ego_state.x + next_ego_state.length / 2.0f * cos(next_ego_state.psi * 180 / PI) - next_ego_state.width / 2.0f * sin(next_ego_state.psi * 180 / PI),
                       next_ego_state.x + next_ego_state.length / 2.0f * sin(next_ego_state.psi * 180 / PI) + next_ego_state.width / 2.0f * cos(next_ego_state.psi * 180 / PI)));

  for (auto const &object_state : next_objects_state)
  {
    Rect2D rect_vehicle(Vec2(object_state.x - object_state.length / 2.0f * cos(object_state.psi * 180 / PI) + object_state.width / 2.0f * sin(object_state.psi * 180 / PI), 
                             object_state.y - object_state.length / 2.0f * sin(object_state.psi * 180 / PI) - object_state.width / 2.0f * cos(object_state.psi * 180 / PI)), 
                        Vec2(object_state.x + object_state.length / 2.0f * cos(object_state.psi * 180 / PI) - object_state.width / 2.0f * sin(object_state.psi * 180 / PI),
                             object_state.x + object_state.length / 2.0f * sin(object_state.psi * 180 / PI) + object_state.width / 2.0f * cos(object_state.psi * 180 / PI)));
    if (intersects(rect_ego, rect_vehicle)){
      is_collied = true;
      break;
    }
  }

  state_status.is_collied = is_collied;

  return state_status;
}

float MCTSState::getAcceleration(const ActionType action) const
{
  float acceleration;
  switch (action)
  {
    case ActionType::DEC:
      acceleration = -1.5f;
      break;
    case ActionType::CON:
      acceleration = 0.f;
      break;
    case ActionType::ACC:
      acceleration = 1.5f;
  }
  return acceleration;
}


std::string MCTSState::getAccelerationType(const ActionType action) const
{
  std::string acceleration_tpye = "";

  switch (action)
  {
    case ActionType::DEC:
      acceleration_tpye = "Deceleration";
      break;
    case ActionType::CON:
      acceleration_tpye = "Keep Velocity";
      break;
    case ActionType::ACC:
      acceleration_tpye = "Acceleration";
      break;
    default:
      assert(0 && "Unknown Action!");
      break;
  }
  return acceleration_tpye;
}

void MCTSState::setReferencePathEgo(std::vector<float>& reference_path_ego)
{   
    mcts_data_handler_.setReferencePathEgo(reference_path_ego);

}

std::vector<float> MCTSState::getReferencePathEgo()
{
  return mcts_data_handler_.getReferencePathEgo();
}

void MCTSState::setWaypointEgo(std::vector<Waypoint>& waypoint_ego)
{   
    mcts_data_handler_.setWaypointEgo(waypoint_ego);

}

std::vector<Waypoint> MCTSState::getWaypointEgo()
{
  return mcts_data_handler_.getWaypointEgo();
}
