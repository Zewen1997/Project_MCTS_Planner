#ifndef MCTS_UTILS_H
#define MCTS_UTILS_H

#include <iostream>
#include <vector>
#include <unordered_map>

enum ActionType : long
{
  DEC             = 0,
  CON             = 1,
  ACC             = 2,
  EMERGENCY_BRAKE = 3,
  FAST_ACC        = 4
};


struct EgoState
{
    float ego_station;
    float x;
    float y;
    float psi;
    float velocity;
    float width;
    float length;
    EgoState(){}
    EgoState(const float in_s, const float in_x, const float in_y, const float in_psi, const float in_velocity, const float in_width, const float in_length): ego_station(in_s), x(in_x), y(in_y), psi(in_psi), velocity(in_velocity), width(in_width), length(in_length){}
    
    EgoState& operator=(const EgoState& other)
    {
        ego_station =other.ego_station;
        x = other.x;
        y = other.y;
        psi = other.psi;
        velocity = other.velocity;
        width = other.width;
        length = other.length;
    }
    void print()
    {
        std::cout << "ego state: " << "x " << x << " y " << y << " station " << ego_station <<std::endl;
    }
};


struct ObjectState
{
    std::vector<float> object_station;
    float x;
    float y;
    float psi;
    float velocity;
    float width;
    float length;
    int id;
    ObjectState(){}
    ObjectState(const std::vector<float> in_s, const float in_x, const float in_y, const float in_psi, const float in_velocity, const float in_width, const float in_length, const int in_id):object_station(in_s), x(in_x), y(in_y), psi(in_psi), velocity(in_velocity), width(in_width), length(in_length), id(in_id){}
    ObjectState& operator=(const ObjectState& other)
    {
        object_station = other.object_station;
        x = other.x;
        y = other.y;
        psi= other.psi;
        velocity = other.velocity;
        width = other.width;
        length = other.length;
        id = other.id;
    }
    void print()
    {
        std::cout << "object state: " << "x " << x << " y " << y << " station " << object_station[0] << " psi " << psi <<std::endl;
    }
    
};

struct StatesStatus
{   
    bool is_reached;
    bool is_collied;
    StatesStatus(){}
    StatesStatus(const bool in_is_reached, const bool in_is_collied):is_reached(in_is_reached), is_collied(in_is_collied){}
    StatesStatus& operator=(const StatesStatus& other)
    {   
        is_reached = other.is_reached;
        is_collied = other.is_collied;
    }
    void print()
    {
        std::cout << "is reached? " << is_reached << std::endl;
        std::cout << "is collied? " << is_collied << std::endl;
    }
};

struct Waypoint
{
    float x;
    float y;
    float station;
    float psi;
    float kappa;
    Waypoint(){}
    Waypoint(const float in_x, const float in_y, const float in_s, const float in_psi, const float in_kappa):x(in_x), y(in_y), station(in_s), psi(in_psi), kappa(in_kappa){}
    Waypoint& operator=(const Waypoint& other)
    {   
        x = other.x;
        y = other.y;
        station = other.station;
        psi = other.psi;
        kappa = other.kappa;
    }
};


struct SortVehicle
{
    ObjectState object_state_;
    float distance_to_ego_;
    SortVehicle(){}
    SortVehicle(const ObjectState& object_state, const float distance_to_ego):object_state_(object_state), distance_to_ego_(distance_to_ego){}
    SortVehicle& operator=(const SortVehicle& other)
    {   
        object_state_ = other.object_state_;
        distance_to_ego_ = other.distance_to_ego_;

    }

};

struct S
{
    std::unordered_map <int, std::vector<float>> s;
    S(){}
    S(const std::unordered_map <int, std::vector<float>> in_s):s(in_s){}

};

struct Route
{
    std::unordered_map <int, std::vector<Waypoint>> route;
    Route(){}
    Route(const std::unordered_map <int, std::vector<Waypoint>> in_route):route(in_route){}

};


class EnvironmentState
{

private:
    // Define MCTS Planner State Space 
    EgoState                 ego_state_;
    std::vector<ObjectState> object_state_;
    StatesStatus             status_;

public:
    EnvironmentState(){}
    EnvironmentState(const EgoState& ego_state, const std::vector<ObjectState>& object_state, const StatesStatus& status)
    {
        ego_state_ = ego_state;
        object_state_ = object_state;
        status_ = status;
    }
    EnvironmentState& operator=(const EnvironmentState&other)
    {
        ego_state_   = other.ego_state_;
        object_state_= other.object_state_;
        status_      = other.status_;
    }
    void print()
    {
        std::cout << "environment state: " << std::endl;
        ego_state_.print();
        object_state_.front().print();
        status_.print();
    }

    void setStatespace(const EgoState& ego_state, const std::vector<ObjectState>& object_state, const StatesStatus& status)
    { 
        ego_state_ = ego_state;
        object_state_ = object_state;
        status_ = status;
    }

    const EgoState& getEgoState() const
    { return ego_state_;}

    const std::vector<ObjectState>& getObjectState() const
    { return object_state_;}

    const StatesStatus& getStatesStatus() const
    { return status_;}

    
};

#endif
