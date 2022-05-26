#include "mcts_data_handler.h"
#include <cmath>
#include <algorithm>

DataHandler::DataHandler(){}

bool comp(const SortVehicle &s1, const SortVehicle &s2){
    return s1.distance_to_ego_ < s2.distance_to_ego_;
}

float DataHandler::distance(float x1, float x2, float y1, float y2){
    float s = (x1-x2) * (x1-x2) + (y1-y2) * (y1-y2);
    return sqrt(s);
}

std::vector<SortVehicle> DataHandler::getNearstVehicles(EgoState& ego, std::vector<ObjectState>& vehicles){
    std::vector<SortVehicle> sort_vehicles;
    for(auto &vehicle : vehicles)
    {
        if (distance(ego.x, ego.y, vehicle.x, vehicle.y) < 30.f){
            SortVehicle vehicle_(vehicle, distance(ego.x, ego.y, vehicle.x, vehicle.y));
            sort_vehicles.push_back(vehicle_);
        }
    }
    sort(sort_vehicles.begin(), sort_vehicles.end(), comp);
    if (sort_vehicles.size() > number_vehicle){
        std::vector<SortVehicle> sort_vehicles_(sort_vehicles.begin(), sort_vehicles.begin() + number_vehicle);
        return sort_vehicles_;
    }
    else{
        return sort_vehicles;
    }
}
