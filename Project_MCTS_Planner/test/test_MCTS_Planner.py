from argparse import Action
import MCTS
import time

print("-------------------- test MCTS lib --------------------")

# Set reference path and waypoints of ego
reference_path_ego = [0, 0.1835, 0.4]
waypoint1 = MCTS.Waypoint(0, 0, 0, 0.9, 0)
waypoint2 = MCTS.Waypoint(0.4, 0.3, 0.1835, 0.88, 0)
waypoint3 = MCTS.Waypoint(0.5, 0.32, 0.4, 0.88, 0)
waypoint_ego = [waypoint1, waypoint2, waypoint3]


# MCTS EgoState
ego_state = MCTS.EgoState(0.16, 0.25, 0, 0, 5, 0.18, 0.5)


"""
objects_state = []
for i in range (10):
    objects_state.append(MCTS.ObjectState([2, 3], i+5, i+5, i+5, i+5, i+5, i+5, i+5))    

print(objects_state[1])
"""


# MCTS ObjectState
vehicle_state = []
vehicle1 = MCTS.ObjectState([0.1, 0.3],0.39, 0.19, 0.0, 5, 0.18, 0.5, 30)
vehicle2 = MCTS.ObjectState([0.1, 0.3],0.39, 0.19, 0.0, 5, 0.18, 0.5, 33)
vehicle_state.append(vehicle1)
vehicle_state.append(vehicle2)

# MCTS StatesStatus
state_status = MCTS.StatesStatus(False, False)

# MCTS EnvironmentState 
mcts_state = MCTS.EnvironmentState(ego_state, vehicle_state,state_status)
mcts_state.print()

# Set reference path and waypoints from vehicles
reference_path_vehicle = {30: MCTS.S({0: [0,0.3,0.6,0.95], 1:[0,0.21,0.58,0.92]}), 33: MCTS.S({0:[0,0.21,0.52,1.03]})}
waypoint4 =  MCTS.Waypoint(0.03, 0, 0, 0.52, 0)
waypoint5 =  MCTS.Waypoint(0.17, 0.16, 0.3, 0.53, 0)
waypoint6 =  MCTS.Waypoint(0.25, 0.33, 0.6, 0.58, 0)
waypoint7 =  MCTS.Waypoint(0.7, 0.52, 0.95, 0.6, 0)
waypoint8 =  MCTS.Waypoint(0.01, 0.01, 0, 0.3, 0)
waypoint9 =  MCTS.Waypoint(0.23, 0.17, 0.21, 0.28, 0)
waypoint10 =  MCTS.Waypoint(0.5, 0.22, 0.58, 0.24, 0)
waypoint11 =  MCTS.Waypoint(0.73, 0.3, 0.92, 0.22, 0)
waypoint12 =  MCTS.Waypoint(0.21, 0.15, 0, 0.25, 0)
waypoint13 =  MCTS.Waypoint(0.4, 0.21, 0.21, 0.23, 0)
waypoint14 =  MCTS.Waypoint(0.67, 0.28, 0.52, 0.2, 0)
waypoint15 =  MCTS.Waypoint(0.98, 0.3, 1.03, 0.18, 0)
waypoint_vehicle = {30: MCTS.Route({0:[waypoint4, waypoint5, waypoint6, waypoint7], 1:[waypoint8, waypoint9, waypoint10, waypoint11]}), 33: MCTS.Route({0:[waypoint12, waypoint13, waypoint14, waypoint15]})}


# MCTS MCTSPlanner 
mcts_planner = MCTS.MCTSPlanner()
print("done init planner", mcts_planner)

# MCTS process functions
print("init:")
mcts_planner.setReferencePathEgo(reference_path_ego)
mcts_planner.setWaypointEgo(waypoint_ego)
mcts_planner.setReferencePathVehicles(reference_path_vehicle)
mcts_planner.setWaypointVehicles(waypoint_vehicle)
mcts_planner.setEnvironmentState(mcts_state)
mcts_planner.processInit()



# Test update states funtion

"""
mcts = MCTS.MCTSState()
mcts.createStartState(mcts_state)
mcts.setReferencePathEgo(reference_path_ego)
mcts.getReferencePathEgo()
mcts.setWaypointEgo(waypoint_ego)
action = MCTS.ActionType(2)
next_state_ego = mcts.calculateNextEgoState(mcts_state, action)
print("-------------- next ego state ---------------------")
next_state_ego.print()
"""

# mcts.setReferencePathVehicles(reference_path_vehicle)
# mcts.setWaypointVehicles(waypoint_vehicle)

"""
vehicle_state = []
vehicle1 = MCTS.ObjectState([0.1, 0.3],0.39, 0.19, 0.0, 5, 0.18, 0.5, 30)
vehicle2 = MCTS.ObjectState([0.1, 0.3],0.39, 0.19, 0.0, 5, 0.18, 0.5, 33)
vehicle_state.append(vehicle1)
vehicle_state.append(vehicle2)
mcts_state = MCTS.EnvironmentState(ego_state, vehicle_state,state_status)
mcts_state.print()
next_state_vehicle = mcts.calculateNextObjectsState(mcts_state, 5)
print("-------------- next vehicle state ---------------------")
for vehicle in next_state_vehicle:
    vehicle.print()

next_status = mcts.calculateNextStatus(ego_state, [vehicle1,vehicle2])
print("-------------- next status ---------------------")
next_status.print()
"""

print("run mcts_planner:")
t1 = time.time()
mcts_planner.processRun()
t2 = time.time()
print("Planner search time", t2 - t1)


