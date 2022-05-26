#ifndef MCTS_DATAHANDLER_H
#define MCTS_DATAHANDLER_H

#include "mcts_utils.h"

enum MethodeToSort : long
{
  NEARST_VEHICLE  = 0
};

enum MethodeToChooseRoute : long
{
  RANDOM       = 0,
  SAMPLE_BASED = 1
};

class DataHandler
{

public:
    int number_vehicle = 5;

    DataHandler();
    float distance(float x1, float x2, float y1, float y2);
    std::vector<SortVehicle> getNearstVehicles(EgoState& ego, std::vector<ObjectState>& vehicles);
    // std::vector<Waypoint> chooseRoute();
};


#endif