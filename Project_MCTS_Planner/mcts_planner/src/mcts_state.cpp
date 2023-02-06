#include "mcts_state.h"
#include "mcts_geometry.h"
#include "ei/2dtypes.hpp"
#include "ei/2dintersection.hpp"
#include "ei/vector.hpp"
#include <algorithm>
#include <cmath>
#include <cassert>
#include <ctime>
#include "mcts_data_handler.h"

using namespace ei;
int seed = 0;

MCTSState::MCTSState(){}
/*
MCTSState::MCTSState(std::shared_ptr<MCTSDataHandler> mcts_data_handler)
{ 
  std::cout<<"null pointer? from DK"<<std::endl;
  if(mcts_data_handler){std::cout<<1<<std::endl;}
  else{std::cout<<2<<std::endl;}
  
  mcts_data_handler_ = mcts_data_handler;

  std::cout<<"null pointer? from DK_"<<std::endl;
  if(mcts_data_handler_){std::cout<<1<<std::endl;}
  else{std::cout<<2<<std::endl;}

  mcts_data_handler_->getReferencePathEgo();
}
*/


MCTSState::MCTSState(const MCTSState &other, std::shared_ptr<MCTSDataHandler> mcts_data_handler)
{
  state_ = other.getState();
  mcts_data_handler_ = mcts_data_handler;

}

MCTSState &MCTSState::operator=(const MCTSState &other)
{
  state_ = other.getState();
  mcts_data_handler_ = other.mcts_data_handler_;
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
     clock_t t1, t2, t3, t4, t5, t6, t7;
     t1 = clock();
     EgoState next_ego_state = this->calculateNextEgoState(state_, action);
     t2 = clock();
     std::cout<< "Run time for calculateNextEgoState(): " << double(t2 - t1) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;

//   // 2. Update next objects state
     t3 = clock();
     std::vector<ObjectState> next_objects_state = this->calculateNextObjectsState(state_, current_depth);
     t4 = clock();
     std::cout<< "Run time for calculateNextObjectsState(): " << double(t4 - t3) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;

//   // 3. Update status between ego and objects
    t5 = clock();
    StatesStatus next_states_status = this->calculateNextStatus(next_ego_state, next_objects_state);
    t6 = clock();
    std::cout<< "Run time for calculateNextStatus(): " << double(t6 - t5) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;
    
    state_.setStatespace(next_ego_state, next_objects_state, next_states_status);
    
    t7 = clock();
    std::cout<< "Run time for applyaction(): " << double(t7 - t1) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;
}

void MCTSState::getActions(std::vector<ActionType> &actions) const
{
  int number_actions = 5;
  // Get all possible actions
  for (long code = 0; code < number_actions; code++)
  {
    switch (code)
    {
      case 0:
        actions.push_back(ActionType::EMERGENCY_BRAKE);
        break;
      case 1:
        actions.push_back(ActionType::DEC);
        break;
      case 2:
        actions.push_back(ActionType::CON);
        break;
      case 3:
        actions.push_back(ActionType::ACC);
        break;
      case 4:
        actions.push_back(ActionType::FAST_ACC);
        break;
      default:
        break;
    }
  }
}

ActionType MCTSState::getRandomAction()
{
  std::vector<ActionType> actions;
  this->getActions(actions);

  // randomize the order
  std::srand(seed);
  seed += 5;
  std::cout<<seed<<std::endl;
  std::random_shuffle(actions.begin(), actions.end());
  for (auto const &a: actions)
  {
    std::cout<<"action: "<< getAcceleration(a)<<std::endl;
  }
  ActionType action = actions.front();
  // std::cout<<"action: "<< getAcceleration(action)<<std::endl;
  return action;
}


const std::vector<float> MCTSState::evaluateRewards(ActionType &action) const
{
  // evalutate rewards based next state (state_ here is already a next state)   
  EgoState            next_ego_state     = state_.getEgoState();
  StatesStatus        next_states_status = state_.getStatesStatus();
  std::vector<float> reward(1, 0.f);

  // Parameters TODO yaml
  float collision_reward     = -2000.f;
  float keep_velocity_reward = -10.f;
  float spped_limit_reward   = -20.f;
  float action_reward        = -0.1f;

  // 1. collision reward
  if (next_states_status.is_collied)
  {
    reward.front() += collision_reward - pow(next_ego_state.velocity, 2);
  }

  // 2. Velocity reward
  
  if (next_ego_state.velocity < 8.33f)
  {
    reward.front() += 3.f * (next_ego_state.velocity - 8.33f);
  }

  else
  {
    if (next_ego_state.velocity > 16.67f)
  {
    reward.front() += 4.f * (next_ego_state.velocity - 16.67f);
  }
  }
  // 3. Action Reward
  if (getAcceleration(action) != 0.f)
  {
    reward.front() += action_reward;
  }
  
  
  /*
  if (next_ego_state.velocity <= 0.5f)
  {
    reward.front() += -50.f;
  }
 
  
  float velocity_desired = 10;
  if (velocity_desired - next_ego_state.velocity >= 0.f)
  {
    reward.front() += keep_velocity_reward * std::fabs(velocity_desired - next_ego_state.velocity);
  }
  else
  {
    reward.front() += spped_limit_reward * std::fabs(velocity_desired - next_ego_state.velocity);
  }
  */

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
  // std::vector<float> reference_path = mcts_data_handler_->getReferencePathEgo();
  // std::vector<Waypoint> waypoint_ego = mcts_data_handler_->getWaypointEgo();


  float   time_step = 0.3f;  // TODO time_step as a parameter
  float   weight;
  
  // Get acceleration value
  const float acceleration = getAcceleration(action);

  std::cout<<"current_ego_state.ego_station: "<<current_ego_state.ego_station<<std::endl;
  if (acceleration < 0.0f && (double)current_ego_state.velocity / (-acceleration) <= time_step)
  {
    time_step = (double)current_ego_state.velocity / (-acceleration);
    std::cout<<"new time_step:" <<time_step<<std::endl;
  }
  next_ego_state.ego_station = current_ego_state.ego_station + current_ego_state.velocity * time_step + 0.5f * acceleration * pow(time_step, 2);
  std::cout<<"next_ego_state.ego_station: "<<next_ego_state.ego_station<<std::endl;
  int index = std::distance(mcts_data_handler_->getReferencePathEgo().begin(), std::lower_bound(mcts_data_handler_->getReferencePathEgo().begin(), mcts_data_handler_->getReferencePathEgo().end(), next_ego_state.ego_station));

  if (index >= mcts_data_handler_->getReferencePathEgo().size()){
    index = mcts_data_handler_->getReferencePathEgo().size() - 1;
  }
  
  if (index != 0 ){
    
    waypoint_right = mcts_data_handler_->getWaypointEgo()[index];
    waypoint_left = mcts_data_handler_->getWaypointEgo()[index - 1];
    weight = (next_ego_state.ego_station - waypoint_left.station) / (waypoint_right.station - waypoint_left.station);
    next_ego_state.x = weight * (waypoint_right.x - waypoint_left.x) + waypoint_left.x;
    next_ego_state.y = weight * (waypoint_right.y - waypoint_left.y) + waypoint_left.y;
    next_ego_state.psi = weight * (waypoint_right.psi - waypoint_left.psi) + waypoint_left.psi;
    next_ego_state.velocity = current_ego_state.velocity + acceleration * time_step;

    
  }
  
  return next_ego_state;
}

std::vector<ObjectState> MCTSState::calculateNextObjectsState(EnvironmentState const& state, const int depth)
{
  clock_t t1, t2, t3, t4, t5, t6, t7;
  t1 = clock();
  std::vector<ObjectState>  next_objects_state;
  std::vector<ObjectState> sorted_objects_state;
  Waypoint waypoint_right;
  Waypoint waypoint_left;

  // std::unordered_map<int, S> reference_path = mcts_data_handler_->getReferencePathVehicles();
  // std::unordered_map<int, Route> route_vehicle = mcts_data_handler_->getWaypointVehicles();
  float time_step = 0.3f;  // TODO time_step as a parameter
  float weight;
  t2 = clock();

  if(state.getObjectState().empty())
  {
    std::cout<<"Object: None!"<<std::endl;
  }
  else{std::cout<<"Object_size: "<<state.getObjectState().size()<<std::endl;}

  if (!state.getObjectState().empty())
  { 
    // calculate each object state of next time step
    EgoState current_ego_state = state.getEgoState();
    std::vector<ObjectState> current_objects_state = state.getObjectState();
    std::vector<SortVehicle> sorted_vehicles = mcts_data_handler_->getNearstVehicles(current_ego_state, current_objects_state);
    
    
    
    for(int i = 0; i < sorted_vehicles.size(); i++)
    {
      ObjectState object_state = sorted_vehicles[i].object_state_;
      sorted_objects_state.emplace_back(object_state);
      
    }

    int j = 0; 
    for (auto const &object : sorted_objects_state)
    { 

      ObjectState next_object_state;
      next_object_state = object;
      std::vector <float> station = object.object_station;
     
      srand(seed);

      int random_route = rand()%(mcts_data_handler_->getWaypointVehicles()[object.id].route.size());
      // Two relatively closed cars will use waypoint to update state, remaining cars use constant speed
      std::cout<<"random:" << random_route <<std::endl;

      if (j <= 1){
        for(float &path: station)
        path += object.velocity * time_step;
        next_object_state.object_station = station;
        
        int index = std::distance(mcts_data_handler_->getReferencePathVehicles()[object.id].s[random_route].begin(), std::lower_bound(mcts_data_handler_->getReferencePathVehicles()[object.id].s[random_route].begin(), mcts_data_handler_->getReferencePathVehicles()[object.id].s[random_route].end(), next_object_state.object_station[random_route]));
 
        if(index != 0){
          /*
          std::cout<<"s_size:"<<reference_path_vehicles[object.id].s[random_route].size()<<std::endl;
          std::cout<<"route_size: "<< route_vehicle[object.id].route.size() <<std::endl;
          std::cout<<"waypoint_size: "<< route_vehicle[object.id].route[random_route].size() <<std::endl;
          */

          waypoint_right = mcts_data_handler_->getWaypointVehicles()[object.id].route[random_route][index];
          waypoint_left = mcts_data_handler_->getWaypointVehicles()[object.id].route[random_route][index - 1];
          weight = (next_object_state.object_station[random_route] - waypoint_left.station) / (waypoint_right.station - waypoint_left.station);
          next_object_state.x = weight * (waypoint_right.x - waypoint_left.x) + waypoint_left.x;
          next_object_state.y = weight * (waypoint_right.y - waypoint_left.y) + waypoint_left.y;
          next_object_state.psi = weight * (waypoint_right.psi - waypoint_left.psi) + waypoint_left.psi;
          next_objects_state.emplace_back(next_object_state);
          /*
          std::cout<<"-----next_object_id-----: "<<next_object_state.id<<std::endl;
          std::cout<<"next_object_x: "<< next_object_state.x <<"next_object_y: "<< next_object_state.y <<std::endl;
          std::cout<<"next_object_ori: "<< next_object_state.psi <<"next_object_length: "<< next_object_state.length <<std::endl;
          std::cout<<"next_object_width: "<<next_object_state.width<<std::endl;
          */
        }

        else
        {  
    
          next_object_state.x = mcts_data_handler_->getWaypointVehicles()[object.id].route[random_route][0].x;
          next_object_state.y = mcts_data_handler_->getWaypointVehicles()[object.id].route[random_route][0].y;
          next_object_state.psi = mcts_data_handler_->getWaypointVehicles()[object.id].route[random_route][0].psi;
          next_objects_state.emplace_back(next_object_state);
          
        }
      }


      else
      {
      next_object_state.psi = object.psi;
      next_object_state.velocity  = object.velocity;
      next_object_state.x   = object.x + time_step * object.velocity * cos(object.psi);
      next_object_state.y   = object.y + time_step * object.velocity * sin(object.psi);
      next_objects_state.emplace_back(next_object_state);

      /*
      std::cout<<"-----next_object_id-----: "<<next_object_state.id<<std::endl;
      std::cout<<"next_object_x: "<< next_object_state.x <<"next_object_y: "<< next_object_state.y <<std::endl;
      std::cout<<"next_object_ori: "<< next_object_state.psi <<"next_object_length: "<< next_object_state.length <<std::endl;
      std::cout<<"next_object_width: "<<next_object_state.width<<std::endl;
      */
      }
      j++;
    }
  }
  t3 = clock();
  std::cout<< "Run time for calc_waypoints: " << double(t3 - t2) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;

  return next_objects_state;
}

StatesStatus MCTSState::calculateNextStatus(EgoState const& next_ego_state, std::vector<ObjectState> const& next_objects_state) const
{
  StatesStatus state_status;
  std::cout<<"Object_size: "<<next_objects_state.size()<<std::endl;
  bool is_collied = false;
  /*
  1. Rect2D with Epsilion-Intersection (Only for AABB, not OBB!)

  Rect2D rect_ego(Vec2(next_ego_state.x - next_ego_state.length / 2.0f * cos(next_ego_state.psi) + next_ego_state.width / 2.0f * sin(next_ego_state.psi), 
                       next_ego_state.y - next_ego_state.length / 2.0f * sin(next_ego_state.psi) - next_ego_state.width / 2.0f * cos(next_ego_state.psi)), 
                  Vec2(next_ego_state.x + next_ego_state.length / 2.0f * cos(next_ego_state.psi) - next_ego_state.width / 2.0f * sin(next_ego_state.psi),
                       next_ego_state.y + next_ego_state.length / 2.0f * sin(next_ego_state.psi) + next_ego_state.width / 2.0f * cos(next_ego_state.psi)));

  
  for (auto const &object_state : next_objects_state)
  {
    Rect2D rect_vehicle(Vec2(object_state.x - object_state.length / 2.0f * cos(object_state.psi) + object_state.width / 2.0f * sin(object_state.psi), 
                             object_state.y - object_state.length / 2.0f * sin(object_state.psi) - object_state.width / 2.0f * cos(object_state.psi)), 
                        Vec2(object_state.x + object_state.length / 2.0f * cos(object_state.psi) - object_state.width / 2.0f * sin(object_state.psi),
   
    
    if (intersects(rect_ego, rect_vehicle)){
      std::cout<<"peng!"<<std::endl;
      is_collied = true;
    }
    break;
  }

  2. Check for collision of two vehicles with the same radius

  float radius = sqrt(pow(next_ego_state.length / 2, 2) + pow(next_ego_state.width / 2, 2));
  for (auto const &object_state : next_objects_state)
  {
    float distance = mcts_data_handler_->distance(next_ego_state.x, object_state.x, next_ego_state.y, object_state.y);
    if (distance <= 2 * radius)
    {
      std::cout<<"peng!"<<std::endl;
      is_collied = true;
      }
    break;
  }
  
  3. Check for collision with third party fcl and libccd

  using OBBPtr_t = std::shared_ptr<collision::RectangleOBB>;
  OBBPtr_t BoxEgo(new collision::RectangleOBB(next_ego_state.width / 2,
                                              next_ego_state.length / 2, 
                                             (next_ego_state.psi * 180 / PI - 90) / 180 * PI,
                                              Eigen::Vector2d(next_ego_state.x, next_ego_state.y)));

  for (auto const &object_state : next_objects_state)
  {
    OBBPtr_t BoxVehicle(new collision::RectangleOBB(object_state.width / 2,
                                                    object_state.length / 2, 
                                                   (object_state.psi * 180 / PI - 90) / 180 * PI,
                                                    Eigen::Vector2d(object_state.x, object_state.y)));
    bool result = BoxEgo -> collide(*BoxVehicle);
    if (result)
    {
      is_collied = true;
    }
    break;
  }
  */
   // 4. Separating Axis Theorem for Oriented Bounding Boxes
  OBB2D obb_ego = OBB2D(Vector2(next_ego_state.x, next_ego_state.y), 
                        next_ego_state.width, 
                        next_ego_state.length, 
                        (next_ego_state.psi * 180 / PI - 90) / 180 * PI);

  for (auto const &object_state : next_objects_state)
  {
    OBB2D obb_vehicle = OBB2D(Vector2(object_state.x, object_state.y), 
                             object_state.width, 
                             object_state.length, 
                             (object_state.psi * 180 / PI - 90) / 180 * PI);

    if (obb_ego.overlaps(obb_vehicle))
    {
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
      acceleration = -2.0f;
      break;
    case ActionType::CON:
      acceleration = 0.f;
      break;
    case ActionType::ACC:
      acceleration = 1.5f;
      break;
    case ActionType::EMERGENCY_BRAKE:
      acceleration = -4.0f;
      break;
    case ActionType::FAST_ACC:
      acceleration = 3.0f;
      break;
  }
  return acceleration;
}


std::string MCTSState::getAccelerationType(const ActionType action) const
{
  std::string acceleration_type = "";

  switch (action)
  {
    case ActionType::DEC:
      acceleration_type = "Deceleration";
      break;
    case ActionType::CON:
      acceleration_type = "Keep Velocity";
      break;
    case ActionType::ACC:
      acceleration_type = "Acceleration";
      break;
    case ActionType::EMERGENCY_BRAKE:
      acceleration_type = "Emergency break";
      break;
    case ActionType::FAST_ACC:
      acceleration_type = "Fast Acceleration";
      break;
    default:
      assert(0 && "Unknown Action!");
      break;
  }
  return acceleration_type;
}

void MCTSState::setReferencePathEgo(std::vector<float>& reference_path_ego)
{   
    mcts_data_handler_->setReferencePathEgo(reference_path_ego);

}

std::vector<float> MCTSState::getReferencePathEgo()
{
  return mcts_data_handler_->getReferencePathEgo();
}

void MCTSState::setWaypointEgo(std::vector<Waypoint>& waypoint_ego)
{   
    mcts_data_handler_->setWaypointEgo(waypoint_ego);

}

std::vector<Waypoint> MCTSState::getWaypointEgo()
{
  return mcts_data_handler_->getWaypointEgo();
}

void MCTSState::setReferencePathVehicles(std::unordered_map<int, S>& reference_path_vehicles)
{   
    mcts_data_handler_->setReferencePathVehicles(reference_path_vehicles);

}

std::unordered_map<int, S> MCTSState::getReferencePathVehicles()
{
  return mcts_data_handler_->getReferencePathVehicles();
}

void MCTSState::setWaypointVehicles(std::unordered_map<int, Route>& route_vehicles)
{   
    mcts_data_handler_->setWaypointVehicles(route_vehicles);

}

std::unordered_map<int, Route> MCTSState::getWaypointVehicles()
{
  return mcts_data_handler_->getWaypointVehicles();
}
/*
void  MCTSState::setDataHandler(std::shared_ptr<MCTSDataHandler>& mcts_data_handler)
{
  mcts_data_handler_ = mcts_data_handler;
}

 std::shared_ptr<MCTSDataHandler>  MCTSState::getDataHandler() const
 {
   return mcts_data_handler_;
 }
 */
