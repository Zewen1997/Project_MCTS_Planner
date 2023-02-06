#include "mcts_data_handler.h"


MCTSDataHandler::MCTSDataHandler()
{
    use_exp3_ = false;
}

// ---------------------------- Interface functions ------------------------ //

void MCTSDataHandler::setEnvironmentState(const EnvironmentState& environment_state)
{
    environment_state_ = environment_state;
    environment_state_.print();
}

EnvironmentState& MCTSDataHandler::getEnvironmentState()
{   
    return environment_state_;
}

void MCTSDataHandler::setDecisions(const std::vector<float>& decisions)
{
    decisions_ = decisions;
    std::cout<<"test: setDecision()"<<std::endl;
    for(auto& decision:decisions_)
    {
        std::cout<<decision<<" "<<std::endl;
    }
}

std::vector<float>& MCTSDataHandler::getDecisions()
{   
    std::cout<<"test: getDecision()"<<std::endl;
    for(auto& decision:decisions_)
    {
        std::cout<<decision<<" "<<std::endl;
    }
    return decisions_;
    

}


void MCTSDataHandler::setReferencePathEgo(const std::vector<float>& reference_path_ego)
{   
    reference_path_ego_ = reference_path_ego;

}

std::vector<float>& MCTSDataHandler::getReferencePathEgo()
{   
    /*
    std::cout<<"test: getReferencePathEgo()"<<std::endl;
    for(auto& reference_path_ego:reference_path_ego_)
    {
        std::cout<<reference_path_ego<<" "<<std::endl;
    }
    */
    return reference_path_ego_;
}

void MCTSDataHandler::setWaypointEgo(const std::vector<Waypoint>& waypoint_ego)
{
    waypoint_ego_ = waypoint_ego;
}

std::vector<Waypoint>& MCTSDataHandler::getWaypointEgo()
{
    return waypoint_ego_;
}

void MCTSDataHandler::setReferencePathVehicles(const std::unordered_map<int, S>& reference_path_vehicles)
{
    reference_path_vehicles_ = reference_path_vehicles;
}

std::unordered_map<int, S>& MCTSDataHandler::getReferencePathVehicles()
{
    return reference_path_vehicles_;
}

void MCTSDataHandler::setWaypointVehicles(const std::unordered_map<int, Route>& route_vehicles)
{
    route_vehicles_ = route_vehicles;
}

std::unordered_map<int, Route>& MCTSDataHandler::getWaypointVehicles()
{
    return route_vehicles_;
}

 void MCTSDataHandler::setExp3(const bool use_exp3)
 {
     use_exp3_ =  use_exp3;
 }

 bool MCTSDataHandler::getExp3()
 {
     return use_exp3_;
 }

// ---------------------------- Data process ------------------------------- //

bool comp(const SortVehicle &s1, const SortVehicle &s2){
    return s1.distance_to_ego_ < s2.distance_to_ego_;
}

float MCTSDataHandler::distance(float x1, float x2, float y1, float y2){
    float s = (x1-x2) * (x1-x2) + (y1-y2) * (y1-y2);
    return sqrt(s);
}

std::vector<SortVehicle> MCTSDataHandler::getNearstVehicles(EgoState& ego, std::vector<ObjectState>& vehicles){
    std::vector<SortVehicle> sort_vehicles;
    for(auto &vehicle : vehicles)
    {   
        // std::cout<<"distance_to_ego: " <<distance(ego.x, vehicle.x, ego.y, vehicle.y)<<std::endl;
        // std::cout<<"vehicle_id_1: "<<vehicle.id<<std::endl;
        if (distance(ego.x, vehicle.x, ego.y, vehicle.y) < 30.f){
            SortVehicle vehicle_(vehicle, distance(ego.x, vehicle.x, ego.y, vehicle.y));
            sort_vehicles.push_back(vehicle_);

        }
    }

    
    sort(sort_vehicles.begin(), sort_vehicles.end(), comp);
    
    if (sort_vehicles.size() > number_vehicle){
        std::vector<SortVehicle> sort_vehicles_(sort_vehicles.begin(), sort_vehicles.begin() + number_vehicle);
        return sort_vehicles_;
        
    }
    else
    {
        return sort_vehicles;
    }

}
